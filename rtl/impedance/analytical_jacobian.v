`timescale 1ns / 1ps

// analytical_jacobian.v - Parallel Analytical Jacobian Calculator
//
// Optimization: all 6 Jacobian columns computed in 1 cycle (was 2 cycles)
// by doing all 12 cross products in parallel.
//
// Total latency: FK (~60 cycles) + dp(1) + J(1) + done(1) = ~63 cycles

module analytical_jacobian (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,

    // Output: end-effector pose
    output wire signed [31:0] pose_x, pose_y, pose_z,
    output wire signed [31:0] pose_rx, pose_ry, pose_rz,

    // Output: 6x6 Jacobian matrix
    output reg  signed [31:0] J00, J01, J02, J03, J04, J05,
    output reg  signed [31:0] J10, J11, J12, J13, J14, J15,
    output reg  signed [31:0] J20, J21, J22, J23, J24, J25,
    output reg  signed [31:0] J30, J31, J32, J33, J34, J35,
    output reg  signed [31:0] J40, J41, J42, J43, J44, J45,
    output reg  signed [31:0] J50, J51, J52, J53, J54, J55,

    output reg         done
);

    // State machine
    localparam S_IDLE     = 3'd0;
    localparam S_WAIT_FK  = 3'd1;
    localparam S_CALC_DP  = 3'd2;  // Compute difference vectors (1 cycle)
    localparam S_CALC_J   = 3'd3;  // Compute ALL 6 Jacobian columns (1 cycle)
    localparam S_DONE     = 3'd4;

    reg [2:0] state;

    // FK engine control
    reg fk_start;
    wire fk_done;

    // FK intermediate frame data
    wire signed [31:0] f0_zx, f0_zy, f0_zz, f0_px, f0_py, f0_pz;
    wire signed [31:0] f1_zx, f1_zy, f1_zz, f1_px, f1_py, f1_pz;
    wire signed [31:0] f2_zx, f2_zy, f2_zz, f2_px, f2_py, f2_pz;
    wire signed [31:0] f3_zx, f3_zy, f3_zz, f3_px, f3_py, f3_pz;
    wire signed [31:0] f4_zx, f4_zy, f4_zz, f4_px, f4_py, f4_pz;

    wire signed [31:0] fk_T00, fk_T01, fk_T02, fk_T03;
    wire signed [31:0] fk_T10, fk_T11, fk_T12, fk_T13;
    wire signed [31:0] fk_T20, fk_T21, fk_T22, fk_T23;
    wire signed [31:0] fk_T30, fk_T31, fk_T32, fk_T33;

    wire signed [31:0] pe_x, pe_y, pe_z;
    assign pe_x = pose_x;
    assign pe_y = pose_y;
    assign pe_z = pose_z;

    // Base frame constants
    localparam signed [31:0] BASE_ZX = 32'sd0;
    localparam signed [31:0] BASE_ZY = 32'sd0;
    localparam signed [31:0] BASE_ZZ = 32'sd65536;

    // FK engine instance (already parallelized)
    fk_engine_ext fk_inst (
        .clk(clk), .rst_n(rst_n), .start(fk_start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .T00(fk_T00), .T01(fk_T01), .T02(fk_T02), .T03(fk_T03),
        .T10(fk_T10), .T11(fk_T11), .T12(fk_T12), .T13(fk_T13),
        .T20(fk_T20), .T21(fk_T21), .T22(fk_T22), .T23(fk_T23),
        .T30(fk_T30), .T31(fk_T31), .T32(fk_T32), .T33(fk_T33),
        .pose_x(pose_x), .pose_y(pose_y), .pose_z(pose_z),
        .pose_rx(pose_rx), .pose_ry(pose_ry), .pose_rz(pose_rz),
        .frame0_zx(f0_zx), .frame0_zy(f0_zy), .frame0_zz(f0_zz),
        .frame0_px(f0_px), .frame0_py(f0_py), .frame0_pz(f0_pz),
        .frame1_zx(f1_zx), .frame1_zy(f1_zy), .frame1_zz(f1_zz),
        .frame1_px(f1_px), .frame1_py(f1_py), .frame1_pz(f1_pz),
        .frame2_zx(f2_zx), .frame2_zy(f2_zy), .frame2_zz(f2_zz),
        .frame2_px(f2_px), .frame2_py(f2_py), .frame2_pz(f2_pz),
        .frame3_zx(f3_zx), .frame3_zy(f3_zy), .frame3_zz(f3_zz),
        .frame3_px(f3_px), .frame3_py(f3_py), .frame3_pz(f3_pz),
        .frame4_zx(f4_zx), .frame4_zy(f4_zy), .frame4_zz(f4_zz),
        .frame4_px(f4_px), .frame4_py(f4_py), .frame4_pz(f4_pz),
        .done(fk_done)
    );

    // Q16.16 fixed-point multiply (DSP48 inferred)
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

    // Difference vectors: dp_i = p_e - p_{i-1}
    reg signed [31:0] dp0_x, dp0_y, dp0_z;
    reg signed [31:0] dp1_x, dp1_y, dp1_z;
    reg signed [31:0] dp2_x, dp2_y, dp2_z;
    reg signed [31:0] dp3_x, dp3_y, dp3_z;
    reg signed [31:0] dp4_x, dp4_y, dp4_z;
    reg signed [31:0] dp5_x, dp5_y, dp5_z;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0;
            state <= S_IDLE;
            fk_start <= 1'b0;
            J00 <= 0; J01 <= 0; J02 <= 0; J03 <= 0; J04 <= 0; J05 <= 0;
            J10 <= 0; J11 <= 0; J12 <= 0; J13 <= 0; J14 <= 0; J15 <= 0;
            J20 <= 0; J21 <= 0; J22 <= 0; J23 <= 0; J24 <= 0; J25 <= 0;
            J30 <= 0; J31 <= 0; J32 <= 0; J33 <= 0; J34 <= 0; J35 <= 0;
            J40 <= 0; J41 <= 0; J42 <= 0; J43 <= 0; J44 <= 0; J45 <= 0;
            J50 <= 0; J51 <= 0; J52 <= 0; J53 <= 0; J54 <= 0; J55 <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        fk_start <= 1'b1;
                        state <= S_WAIT_FK;
                    end
                end

                S_WAIT_FK: begin
                    fk_start <= 1'b0;
                    if (fk_done) begin
                        // Compute ALL 6 difference vectors in parallel
                        dp0_x <= pe_x;
                        dp0_y <= pe_y;
                        dp0_z <= pe_z;
                        dp1_x <= pe_x - f0_px;
                        dp1_y <= pe_y - f0_py;
                        dp1_z <= pe_z - f0_pz;
                        dp2_x <= pe_x - f1_px;
                        dp2_y <= pe_y - f1_py;
                        dp2_z <= pe_z - f1_pz;
                        dp3_x <= pe_x - f2_px;
                        dp3_y <= pe_y - f2_py;
                        dp3_z <= pe_z - f2_pz;
                        dp4_x <= pe_x - f3_px;
                        dp4_y <= pe_y - f3_py;
                        dp4_z <= pe_z - f3_pz;
                        dp5_x <= pe_x - f4_px;
                        dp5_y <= pe_y - f4_py;
                        dp5_z <= pe_z - f4_pz;
                        state <= S_CALC_J;
                    end
                end

                // ==========================================================
                // ALL 6 Jacobian columns in 1 cycle (was 2 cycles)
                // 30 multipliers for 10 cross products (col 0 is trivial)
                // ==========================================================
                S_CALC_J: begin
                    // Column 0: z_{-1}=[0,0,1] × dp0 (trivial cross product)
                    J00 <= -dp0_y;
                    J10 <= dp0_x;
                    J20 <= 32'sd0;
                    J30 <= BASE_ZX;
                    J40 <= BASE_ZY;
                    J50 <= BASE_ZZ;

                    // Column 1: z_0 × dp1
                    J01 <= fmult(f0_zy, dp1_z) - fmult(f0_zz, dp1_y);
                    J11 <= fmult(f0_zz, dp1_x) - fmult(f0_zx, dp1_z);
                    J21 <= fmult(f0_zx, dp1_y) - fmult(f0_zy, dp1_x);
                    J31 <= f0_zx;
                    J41 <= f0_zy;
                    J51 <= f0_zz;

                    // Column 2: z_1 × dp2
                    J02 <= fmult(f1_zy, dp2_z) - fmult(f1_zz, dp2_y);
                    J12 <= fmult(f1_zz, dp2_x) - fmult(f1_zx, dp2_z);
                    J22 <= fmult(f1_zx, dp2_y) - fmult(f1_zy, dp2_x);
                    J32 <= f1_zx;
                    J42 <= f1_zy;
                    J52 <= f1_zz;

                    // Column 3: z_2 × dp3
                    J03 <= fmult(f2_zy, dp3_z) - fmult(f2_zz, dp3_y);
                    J13 <= fmult(f2_zz, dp3_x) - fmult(f2_zx, dp3_z);
                    J23 <= fmult(f2_zx, dp3_y) - fmult(f2_zy, dp3_x);
                    J33 <= f2_zx;
                    J43 <= f2_zy;
                    J53 <= f2_zz;

                    // Column 4: z_3 × dp4
                    J04 <= fmult(f3_zy, dp4_z) - fmult(f3_zz, dp4_y);
                    J14 <= fmult(f3_zz, dp4_x) - fmult(f3_zx, dp4_z);
                    J24 <= fmult(f3_zx, dp4_y) - fmult(f3_zy, dp4_x);
                    J34 <= f3_zx;
                    J44 <= f3_zy;
                    J54 <= f3_zz;

                    // Column 5: z_4 × dp5
                    J05 <= fmult(f4_zy, dp5_z) - fmult(f4_zz, dp5_y);
                    J15 <= fmult(f4_zz, dp5_x) - fmult(f4_zx, dp5_z);
                    J25 <= fmult(f4_zx, dp5_y) - fmult(f4_zy, dp5_x);
                    J35 <= f4_zx;
                    J45 <= f4_zy;
                    J55 <= f4_zz;

                    state <= S_DONE;
                end

                S_DONE: begin
                    done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
