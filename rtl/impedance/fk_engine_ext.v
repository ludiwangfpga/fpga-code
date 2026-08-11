`timescale 1ns / 1ps

// fk_engine_ext.v - Massively Parallel Forward Kinematics Engine
//
// Optimizations vs original serial design:
//   1. 6× CORDIC instances: pre-compute all sin(q0)~sin(q5) in parallel (20 cyc)
//   2. Alpha sin/cos as compile-time constants (FREE, saves 6×20=120 cyc)
//   3. Matrix multiply: all 4 rows in 1 cycle (saves 3 cyc/joint)
//   4. 3× atan2 instances: parallel Euler angle extraction (saves 40 cyc)
//
// Cycle budget:
//   Pre-compute trig:    20 cycles (6 CORDICs in parallel)
//   Chain multiply:      6 joints × 3 cyc (Ti+mult+writeback) = 18 cycles
//   Euler angles:        ~22 cycles (2 atan2 parallel + 1 dependent)
//   ─────────────────────────────────────────────────────
//   Total FK:            ~60 cycles (was ~348)  →  5.8× speedup
//
// LUT increase: +5 CORDIC (~1000 LUT) + 4-row parallel mult (~500 LUT)
//             + 2 extra atan2 (~400 LUT) = ~1900 additional LUT

module fk_engine_ext (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,

    // Final 4x4 transform
    output reg  signed [31:0] T00, T01, T02, T03,
    output reg  signed [31:0] T10, T11, T12, T13,
    output reg  signed [31:0] T20, T21, T22, T23,
    output reg  signed [31:0] T30, T31, T32, T33,

    // End-effector pose
    output reg  signed [31:0] pose_x, pose_y, pose_z,
    output reg  signed [31:0] pose_rx, pose_ry, pose_rz,

    // Intermediate frame data for Jacobian
    output reg  signed [31:0] frame0_zx, frame0_zy, frame0_zz,
    output reg  signed [31:0] frame1_zx, frame1_zy, frame1_zz,
    output reg  signed [31:0] frame2_zx, frame2_zy, frame2_zz,
    output reg  signed [31:0] frame3_zx, frame3_zy, frame3_zz,
    output reg  signed [31:0] frame4_zx, frame4_zy, frame4_zz,
    output reg  signed [31:0] frame0_px, frame0_py, frame0_pz,
    output reg  signed [31:0] frame1_px, frame1_py, frame1_pz,
    output reg  signed [31:0] frame2_px, frame2_py, frame2_pz,
    output reg  signed [31:0] frame3_px, frame3_py, frame3_pz,
    output reg  signed [31:0] frame4_px, frame4_py, frame4_pz,

    output reg         done
);

    // =====================================================================
    // State Machine
    // =====================================================================
    localparam S_IDLE       = 4'd0;
    localparam S_TRIG_START = 4'd1;   // Launch 6 CORDICs in parallel
    localparam S_TRIG_WAIT  = 4'd2;   // Wait for all 6 CORDICs
    localparam S_CHAIN_Ti   = 4'd3;   // Build Ti for current joint
    localparam S_CHAIN_MULT = 4'd4;   // 4-row parallel multiply
    localparam S_CHAIN_WB   = 4'd5;   // Write back Ta ← Tt, save frame
    localparam S_EXTRACT    = 4'd6;   // Start Euler angle atan2s
    localparam S_ATAN_WAIT1 = 4'd7;   // Wait for parallel rz & rx atan2
    localparam S_ATAN_WAIT2 = 4'd8;   // Wait for ry atan2 (depends on rz magnitude)
    localparam S_DONE       = 4'd9;

    reg [3:0] state;
    reg [2:0] joint_idx;

    // =====================================================================
    // Q16.16 Constants
    // =====================================================================
    localparam signed [31:0] FIXED_ONE  = 32'sd65536;
    localparam signed [31:0] FIXED_ZERO = 32'sd0;
    localparam signed [31:0] FIXED_NEG_ONE = -32'sd65536;

    // DH alpha sin/cos CONSTANTS (no CORDIC needed!)
    // Joint 0: α=π/2   → sin=1,  cos=0
    // Joint 1: α=0     → sin=0,  cos=1
    // Joint 2: α=π/2   → sin=1,  cos=0
    // Joint 3: α=-π/2  → sin=-1, cos=0
    // Joint 4: α=π/2   → sin=1,  cos=0
    // Joint 5: α=0     → sin=0,  cos=1
    wire signed [31:0] sin_alpha_const [0:5];
    wire signed [31:0] cos_alpha_const [0:5];
    assign sin_alpha_const[0] = FIXED_ONE;
    assign cos_alpha_const[0] = FIXED_ZERO;
    assign sin_alpha_const[1] = FIXED_ZERO;
    assign cos_alpha_const[1] = FIXED_ONE;
    assign sin_alpha_const[2] = FIXED_ONE;
    assign cos_alpha_const[2] = FIXED_ZERO;
    assign sin_alpha_const[3] = FIXED_NEG_ONE;
    assign cos_alpha_const[3] = FIXED_ZERO;
    assign sin_alpha_const[4] = FIXED_ONE;
    assign cos_alpha_const[4] = FIXED_ZERO;
    assign sin_alpha_const[5] = FIXED_ZERO;
    assign cos_alpha_const[5] = FIXED_ONE;

    // DH d and a constants
    wire signed [31:0] dh_d_const [0:5];
    wire signed [31:0] dh_a_const [0:5];
    assign dh_d_const[0] = 32'sd10879;  // 0.166m
    assign dh_d_const[1] = 32'sd0;
    assign dh_d_const[2] = 32'sd0;
    assign dh_d_const[3] = 32'sd12583;  // 0.192m
    assign dh_d_const[4] = 32'sd0;
    assign dh_d_const[5] = 32'sd3604;   // 0.055m
    assign dh_a_const[0] = 32'sd3604;   // 0.055m
    assign dh_a_const[1] = 32'sd13107;  // 0.200m
    assign dh_a_const[2] = 32'sd3670;   // 0.056m
    assign dh_a_const[3] = 32'sd0;
    assign dh_a_const[4] = 32'sd0;
    assign dh_a_const[5] = 32'sd0;

    // =====================================================================
    // 6× Parallel CORDIC sincos — pre-compute all sin(qi)/cos(qi)
    // =====================================================================
    reg cordic_start_all;
    wire [5:0] cordic_done_vec;
    wire cordic_all_done = &cordic_done_vec;

    reg  signed [31:0] q_arr [0:5];
    wire signed [31:0] sin_q_arr [0:5];
    wire signed [31:0] cos_q_arr [0:5];

    genvar gi;
    generate
        for (gi = 0; gi < 6; gi = gi + 1) begin : CORDIC_BANK
            cordic_sincos cordic_q_inst (
                .clk(clk),
                .rst_n(rst_n),
                .start(cordic_start_all),
                .angle(q_arr[gi]),
                .sin_out(sin_q_arr[gi]),
                .cos_out(cos_q_arr[gi]),
                .done(cordic_done_vec[gi])
            );
        end
    endgenerate

    // =====================================================================
    // 3× Parallel CORDIC atan2 — Euler angle extraction
    // =====================================================================
    // atan2_rz: atan2(T10, T00) → rz
    // atan2_rx: atan2(T21, T22) → rx
    // atan2_ry: atan2(-T20, mag) → ry (depends on rz's magnitude output)

    reg atan_start_rz_rx;  // Start rz and rx in parallel
    reg atan_start_ry;     // Start ry after rz done (needs magnitude)

    reg  signed [31:0] atan_rz_x, atan_rz_y;
    wire signed [31:0] atan_rz_angle, atan_rz_mag;
    wire atan_rz_done;

    reg  signed [31:0] atan_rx_x, atan_rx_y;
    wire signed [31:0] atan_rx_angle, atan_rx_mag;
    wire atan_rx_done;

    reg  signed [31:0] atan_ry_x, atan_ry_y;
    wire signed [31:0] atan_ry_angle, atan_ry_mag;
    wire atan_ry_done;

    cordic_atan2 atan2_rz (
        .clk(clk), .rst_n(rst_n), .start(atan_start_rz_rx),
        .x_in(atan_rz_x), .y_in(atan_rz_y),
        .angle(atan_rz_angle), .magnitude(atan_rz_mag), .done(atan_rz_done)
    );

    cordic_atan2 atan2_rx (
        .clk(clk), .rst_n(rst_n), .start(atan_start_rz_rx),
        .x_in(atan_rx_x), .y_in(atan_rx_y),
        .angle(atan_rx_angle), .magnitude(atan_rx_mag), .done(atan_rx_done)
    );

    cordic_atan2 atan2_ry (
        .clk(clk), .rst_n(rst_n), .start(atan_start_ry),
        .x_in(atan_ry_x), .y_in(atan_ry_y),
        .angle(atan_ry_angle), .magnitude(atan_ry_mag), .done(atan_ry_done)
    );

    // =====================================================================
    // Q16.16 fixed-point multiply (DSP48 inferred)
    // =====================================================================
    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [63:0] temp;
        begin
            temp = $signed(a) * $signed(b);
            fmult = temp[47:16];
        end
    endfunction

    // =====================================================================
    // Working registers
    // =====================================================================

    // Current joint's pre-fetched trig and DH
    reg signed [31:0] cur_sin_q, cur_cos_q;
    reg signed [31:0] cur_sin_a, cur_cos_a;
    reg signed [31:0] cur_dh_d, cur_dh_a;

    // Ti matrix (built combinationally from trig + DH)
    reg signed [31:0] Ti00, Ti01, Ti02, Ti03;
    reg signed [31:0] Ti10, Ti11, Ti12, Ti13;
    reg signed [31:0] Ti20, Ti21, Ti22, Ti23;

    // Accumulated T matrix
    reg signed [31:0] Ta00, Ta01, Ta02, Ta03;
    reg signed [31:0] Ta10, Ta11, Ta12, Ta13;
    reg signed [31:0] Ta20, Ta21, Ta22, Ta23;
    reg signed [31:0] Ta30, Ta31, Ta32, Ta33;

    // Temp for 4-row parallel multiply
    reg signed [31:0] Tt00, Tt01, Tt02, Tt03;
    reg signed [31:0] Tt10, Tt11, Tt12, Tt13;
    reg signed [31:0] Tt20, Tt21, Tt22, Tt23;
    reg signed [31:0] Tt30, Tt31, Tt32, Tt33;

    // =====================================================================
    // Main FSM
    // =====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0;
            state <= S_IDLE;
            joint_idx <= 3'd0;
            cordic_start_all <= 1'b0;
            atan_start_rz_rx <= 1'b0;
            atan_start_ry <= 1'b0;

            Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
            Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
            Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
            Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;

            T00 <= 0; T01 <= 0; T02 <= 0; T03 <= 0;
            T10 <= 0; T11 <= 0; T12 <= 0; T13 <= 0;
            T20 <= 0; T21 <= 0; T22 <= 0; T23 <= 0;
            T30 <= 0; T31 <= 0; T32 <= 0; T33 <= 0;
            pose_x <= 0; pose_y <= 0; pose_z <= 0;
            pose_rx <= 0; pose_ry <= 0; pose_rz <= 0;

            frame0_zx <= 0; frame0_zy <= 0; frame0_zz <= 0;
            frame1_zx <= 0; frame1_zy <= 0; frame1_zz <= 0;
            frame2_zx <= 0; frame2_zy <= 0; frame2_zz <= 0;
            frame3_zx <= 0; frame3_zy <= 0; frame3_zz <= 0;
            frame4_zx <= 0; frame4_zy <= 0; frame4_zz <= 0;
            frame0_px <= 0; frame0_py <= 0; frame0_pz <= 0;
            frame1_px <= 0; frame1_py <= 0; frame1_pz <= 0;
            frame2_px <= 0; frame2_py <= 0; frame2_pz <= 0;
            frame3_px <= 0; frame3_py <= 0; frame3_pz <= 0;
            frame4_px <= 0; frame4_py <= 0; frame4_pz <= 0;
        end else begin
            // Default: deassert start signals
            cordic_start_all <= 1'b0;
            atan_start_rz_rx <= 1'b0;
            atan_start_ry <= 1'b0;

            case (state)
                // ==========================================================
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Latch joint angles for 6 parallel CORDICs
                        q_arr[0] <= q0; q_arr[1] <= q1; q_arr[2] <= q2;
                        q_arr[3] <= q3; q_arr[4] <= q4; q_arr[5] <= q5;
                        // Init T_acc to identity
                        Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
                        Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
                        Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
                        Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;
                        state <= S_TRIG_START;
                    end
                end

                // ==========================================================
                // Launch 6 CORDICs in parallel for sin/cos(q0)..sin/cos(q5)
                // ==========================================================
                S_TRIG_START: begin
                    cordic_start_all <= 1'b1;
                    state <= S_TRIG_WAIT;
                end

                S_TRIG_WAIT: begin
                    if (cordic_all_done) begin
                        // All 6 sin/cos ready — start chain multiply
                        joint_idx <= 3'd0;
                        // Pre-fetch joint 0 data
                        cur_sin_q <= sin_q_arr[0];
                        cur_cos_q <= cos_q_arr[0];
                        cur_sin_a <= sin_alpha_const[0];
                        cur_cos_a <= cos_alpha_const[0];
                        cur_dh_d  <= dh_d_const[0];
                        cur_dh_a  <= dh_a_const[0];
                        state <= S_CHAIN_Ti;
                    end
                end

                // ==========================================================
                // Build Ti matrix from pre-computed trig + DH constants
                // ==========================================================
                S_CHAIN_Ti: begin
                    // DH transformation matrix:
                    // [cos_q   -sin_q*cos_a   sin_q*sin_a   a*cos_q ]
                    // [sin_q    cos_q*cos_a  -cos_q*sin_a   a*sin_q ]
                    // [0        sin_a         cos_a          d       ]
                    // [0        0             0              1       ]
                    Ti00 <= cur_cos_q;
                    Ti01 <= -fmult(cur_sin_q, cur_cos_a);
                    Ti02 <= fmult(cur_sin_q, cur_sin_a);
                    Ti03 <= fmult(cur_dh_a, cur_cos_q);
                    Ti10 <= cur_sin_q;
                    Ti11 <= fmult(cur_cos_q, cur_cos_a);
                    Ti12 <= -fmult(cur_cos_q, cur_sin_a);
                    Ti13 <= fmult(cur_dh_a, cur_sin_q);
                    Ti20 <= FIXED_ZERO;
                    Ti21 <= cur_sin_a;
                    Ti22 <= cur_cos_a;
                    Ti23 <= cur_dh_d;
                    state <= S_CHAIN_MULT;
                end

                // ==========================================================
                // 4-row PARALLEL matrix multiply: Tt = Ta × Ti
                // All 4 rows computed in a single cycle!
                // Uses 64 multipliers (4 rows × 4 cols × 4 dot-product terms)
                // ==========================================================
                S_CHAIN_MULT: begin
                    // Row 0
                    Tt00 <= fmult(Ta00,Ti00) + fmult(Ta01,Ti10);
                    Tt01 <= fmult(Ta00,Ti01) + fmult(Ta01,Ti11) + fmult(Ta02,Ti21);
                    Tt02 <= fmult(Ta00,Ti02) + fmult(Ta01,Ti12) + fmult(Ta02,Ti22);
                    Tt03 <= fmult(Ta00,Ti03) + fmult(Ta01,Ti13) + fmult(Ta02,Ti23) + Ta03;
                    // Row 1
                    Tt10 <= fmult(Ta10,Ti00) + fmult(Ta11,Ti10);
                    Tt11 <= fmult(Ta10,Ti01) + fmult(Ta11,Ti11) + fmult(Ta12,Ti21);
                    Tt12 <= fmult(Ta10,Ti02) + fmult(Ta11,Ti12) + fmult(Ta12,Ti22);
                    Tt13 <= fmult(Ta10,Ti03) + fmult(Ta11,Ti13) + fmult(Ta12,Ti23) + Ta13;
                    // Row 2
                    Tt20 <= fmult(Ta20,Ti00) + fmult(Ta21,Ti10);
                    Tt21 <= fmult(Ta20,Ti01) + fmult(Ta21,Ti11) + fmult(Ta22,Ti21);
                    Tt22 <= fmult(Ta20,Ti02) + fmult(Ta21,Ti12) + fmult(Ta22,Ti22);
                    Tt23 <= fmult(Ta20,Ti03) + fmult(Ta21,Ti13) + fmult(Ta22,Ti23) + Ta23;
                    // Row 3: always [0, 0, 0, 1] (no computation needed)
                    Tt30 <= FIXED_ZERO;
                    Tt31 <= FIXED_ZERO;
                    Tt32 <= FIXED_ZERO;
                    Tt33 <= FIXED_ONE;
                    state <= S_CHAIN_WB;
                end

                // ==========================================================
                // Write back Tt → Ta, save intermediate frame, advance joint
                // ==========================================================
                S_CHAIN_WB: begin
                    // Update accumulated transform
                    Ta00 <= Tt00; Ta01 <= Tt01; Ta02 <= Tt02; Ta03 <= Tt03;
                    Ta10 <= Tt10; Ta11 <= Tt11; Ta12 <= Tt12; Ta13 <= Tt13;
                    Ta20 <= Tt20; Ta21 <= Tt21; Ta22 <= Tt22; Ta23 <= Tt23;
                    Ta30 <= Tt30; Ta31 <= Tt31; Ta32 <= Tt32; Ta33 <= Tt33;

                    // Save intermediate frame for Jacobian (frames 0-4)
                    case (joint_idx)
                        3'd0: begin
                            frame0_zx <= Tt02; frame0_zy <= Tt12; frame0_zz <= Tt22;
                            frame0_px <= Tt03; frame0_py <= Tt13; frame0_pz <= Tt23;
                        end
                        3'd1: begin
                            frame1_zx <= Tt02; frame1_zy <= Tt12; frame1_zz <= Tt22;
                            frame1_px <= Tt03; frame1_py <= Tt13; frame1_pz <= Tt23;
                        end
                        3'd2: begin
                            frame2_zx <= Tt02; frame2_zy <= Tt12; frame2_zz <= Tt22;
                            frame2_px <= Tt03; frame2_py <= Tt13; frame2_pz <= Tt23;
                        end
                        3'd3: begin
                            frame3_zx <= Tt02; frame3_zy <= Tt12; frame3_zz <= Tt22;
                            frame3_px <= Tt03; frame3_py <= Tt13; frame3_pz <= Tt23;
                        end
                        3'd4: begin
                            frame4_zx <= Tt02; frame4_zy <= Tt12; frame4_zz <= Tt22;
                            frame4_px <= Tt03; frame4_py <= Tt13; frame4_pz <= Tt23;
                        end
                        default: ;
                    endcase

                    if (joint_idx == 3'd5) begin
                        // FK chain complete — copy to output and extract Euler angles
                        T00 <= Tt00; T01 <= Tt01; T02 <= Tt02; T03 <= Tt03;
                        T10 <= Tt10; T11 <= Tt11; T12 <= Tt12; T13 <= Tt13;
                        T20 <= Tt20; T21 <= Tt21; T22 <= Tt22; T23 <= Tt23;
                        T30 <= Tt30; T31 <= Tt31; T32 <= Tt32; T33 <= Tt33;
                        pose_x <= Tt03;
                        pose_y <= Tt13;
                        pose_z <= Tt23;
                        state <= S_EXTRACT;
                    end else begin
                        // Pre-fetch next joint data (overlapped with writeback)
                        cur_sin_q <= sin_q_arr[joint_idx + 1];
                        cur_cos_q <= cos_q_arr[joint_idx + 1];
                        cur_sin_a <= sin_alpha_const[joint_idx + 1];
                        cur_cos_a <= cos_alpha_const[joint_idx + 1];
                        cur_dh_d  <= dh_d_const[joint_idx + 1];
                        cur_dh_a  <= dh_a_const[joint_idx + 1];
                        joint_idx <= joint_idx + 1;
                        state <= S_CHAIN_Ti;
                    end
                end

                // ==========================================================
                // Euler angle extraction: launch 2 atan2 in parallel
                //   rz = atan2(T10, T00)   — atan2_rz (also gives magnitude)
                //   rx = atan2(T21, T22)   — atan2_rx
                // ==========================================================
                S_EXTRACT: begin
                    // rz: atan2(T10, T00)
                    atan_rz_x <= Ta00;  // Use Ta since we just wrote it
                    atan_rz_y <= Ta10;
                    // rx: atan2(T21, T22)
                    atan_rx_x <= Ta22;
                    atan_rx_y <= Ta21;
                    atan_start_rz_rx <= 1'b1;
                    state <= S_ATAN_WAIT1;
                end

                // ==========================================================
                // Wait for rz & rx (parallel), then launch ry
                //   ry = atan2(-T20, sqrt(T00²+T10²))
                //      where sqrt(T00²+T10²) ≈ magnitude from rz atan2
                // ==========================================================
                S_ATAN_WAIT1: begin
                    if (atan_rz_done && atan_rx_done) begin
                        pose_rz <= atan_rz_angle;
                        pose_rx <= atan_rx_angle;
                        // ry: atan2(-T20, mag)
                        atan_ry_x <= atan_rz_mag;
                        atan_ry_y <= -Ta20;
                        atan_start_ry <= 1'b1;
                        state <= S_ATAN_WAIT2;
                    end
                end

                S_ATAN_WAIT2: begin
                    if (atan_ry_done) begin
                        pose_ry <= atan_ry_angle;
                        state <= S_DONE;
                    end
                end

                // ==========================================================
                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
