`timescale 1ns / 1ps

// fk_engine.v - Forward Kinematics Engine (Verilog-2001 compatible)
// Computes end-effector pose from joint angles using DH convention

module fk_engine (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    // Joint angles (Q16.16 radians) - expanded
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,
    // Output transformation matrix (4x4, row-major, Q16.16) - expanded
    output reg  signed [31:0] T00, T01, T02, T03,
    output reg  signed [31:0] T10, T11, T12, T13,
    output reg  signed [31:0] T20, T21, T22, T23,
    output reg  signed [31:0] T30, T31, T32, T33,
    // Output pose (x, y, z, rx, ry, rz in Q16.16) - expanded
    output reg  signed [31:0] pose_x, pose_y, pose_z,
    output reg  signed [31:0] pose_rx, pose_ry, pose_rz,
    // Intermediate frame data for Jacobian (z-axis and origin per joint 0..4)
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

    // State machine
    localparam IDLE        = 4'd0;
    localparam LOAD_DH     = 4'd1;
    localparam CALC_TRIG   = 4'd2;
    localparam WAIT_TRIG   = 4'd3;
    localparam CALC_Ti     = 4'd4;
    localparam MULT_MATRIX = 4'd5;
    localparam NEXT_JOINT  = 4'd6;
    localparam EXTRACT_POSE= 4'd7;
    localparam WAIT_ATAN1  = 4'd8;
    localparam WAIT_ATAN2  = 4'd9;
    localparam WAIT_ATAN3  = 4'd10;
    localparam DONE_ST     = 4'd11;

    reg [3:0] state;
    reg [2:0] joint_idx;
    reg [3:0] mult_step;

    // Pipelined matrix-multiply product registers: MUL stage registers 16 products,
    // ADD stage sums them -> Tt. Splits multiply and add tree across cycles to
    // close 125MHz timing (combinational fmult + 3-add tree was ~-0.12ns WNS).
    reg signed [31:0] pp0,pp1,pp2,pp3,pp4,pp5,pp6,pp7,pp8,pp9,pp10,pp11,pp12,pp13,pp14,pp15;

    // Joint angle storage
    reg signed [31:0] q_arr [0:5];

    // DH parameters for current joint
    reg signed [31:0] dh_d, dh_a, dh_alpha;
    wire signed [31:0] dh_d_w, dh_a_w, dh_alpha_w;
    wire signed [31:0] q_min_w, q_max_w;

    // CORDIC interface
    reg cordic_start;
    reg signed [31:0] cordic_angle;
    wire signed [31:0] cordic_sin, cordic_cos;
    wire cordic_done;

    // Trigonometric values
    reg signed [31:0] sin_q, cos_q;
    reg signed [31:0] sin_alpha, cos_alpha;
    reg trig_phase;

    // Individual transformation matrix Ti (4x4)
    reg signed [31:0] Ti00, Ti01, Ti02, Ti03;
    reg signed [31:0] Ti10, Ti11, Ti12, Ti13;
    reg signed [31:0] Ti20, Ti21, Ti22, Ti23;
    reg signed [31:0] Ti30, Ti31, Ti32, Ti33;

    // Accumulated transformation matrix T (4x4)
    reg signed [31:0] Ta00, Ta01, Ta02, Ta03;
    reg signed [31:0] Ta10, Ta11, Ta12, Ta13;
    reg signed [31:0] Ta20, Ta21, Ta22, Ta23;
    reg signed [31:0] Ta30, Ta31, Ta32, Ta33;

    // Temporary matrix for multiplication
    reg signed [31:0] Tt00, Tt01, Tt02, Tt03;
    reg signed [31:0] Tt10, Tt11, Tt12, Tt13;
    reg signed [31:0] Tt20, Tt21, Tt22, Tt23;
    reg signed [31:0] Tt30, Tt31, Tt32, Tt33;

    // CORDIC atan2 for pose extraction
    reg atan_start;
    reg signed [31:0] atan_x, atan_y;
    wire signed [31:0] atan_angle, atan_mag;
    wire atan_done;

    // DH configuration module
    part1_config_comb dh_config (
        .joint_idx(joint_idx),
        .dh_d(dh_d_w),
        .dh_a(dh_a_w),
        .dh_alpha(dh_alpha_w),
        .q_min(q_min_w),
        .q_max(q_max_w)
    );

    // CORDIC sincos module
    cordic_sincos cordic_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(cordic_start),
        .angle(cordic_angle),
        .sin_out(cordic_sin),
        .cos_out(cordic_cos),
        .done(cordic_done)
    );

    // CORDIC atan2 module
    cordic_atan2 atan2_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(atan_start),
        .x_in(atan_x),
        .y_in(atan_y),
        .angle(atan_angle),
        .magnitude(atan_mag),
        .done(atan_done)
    );

    // Fixed point multiplication (round-to-nearest: +2^15 before >>16)
    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [63:0] temp;
        begin
            temp = $signed(a) * $signed(b) + 64'sd32768;
            fmult = temp[47:16];
        end
    endfunction

    // Constants
    localparam signed [31:0] FIXED_ONE  = 32'sd65536;
    localparam signed [31:0] FIXED_ZERO = 32'sd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 1'b0;
            state <= IDLE;
            joint_idx <= 3'd0;
            cordic_start <= 1'b0;
            atan_start <= 1'b0;
            trig_phase <= 1'b0;
            mult_step <= 4'd0;

            // Initialize T_acc to identity
            Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
            Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
            Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
            Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;

            // Initialize outputs
            T00 <= FIXED_ZERO; T01 <= FIXED_ZERO; T02 <= FIXED_ZERO; T03 <= FIXED_ZERO;
            T10 <= FIXED_ZERO; T11 <= FIXED_ZERO; T12 <= FIXED_ZERO; T13 <= FIXED_ZERO;
            T20 <= FIXED_ZERO; T21 <= FIXED_ZERO; T22 <= FIXED_ZERO; T23 <= FIXED_ZERO;
            T30 <= FIXED_ZERO; T31 <= FIXED_ZERO; T32 <= FIXED_ZERO; T33 <= FIXED_ZERO;
            pose_x <= FIXED_ZERO; pose_y <= FIXED_ZERO; pose_z <= FIXED_ZERO;
            pose_rx <= FIXED_ZERO; pose_ry <= FIXED_ZERO; pose_rz <= FIXED_ZERO;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Store joint angles
                        q_arr[0] <= q0; q_arr[1] <= q1; q_arr[2] <= q2;
                        q_arr[3] <= q3; q_arr[4] <= q4; q_arr[5] <= q5;
                        joint_idx <= 3'd0;
                        // Initialize T_acc to identity
                        Ta00 <= FIXED_ONE;  Ta01 <= FIXED_ZERO; Ta02 <= FIXED_ZERO; Ta03 <= FIXED_ZERO;
                        Ta10 <= FIXED_ZERO; Ta11 <= FIXED_ONE;  Ta12 <= FIXED_ZERO; Ta13 <= FIXED_ZERO;
                        Ta20 <= FIXED_ZERO; Ta21 <= FIXED_ZERO; Ta22 <= FIXED_ONE;  Ta23 <= FIXED_ZERO;
                        Ta30 <= FIXED_ZERO; Ta31 <= FIXED_ZERO; Ta32 <= FIXED_ZERO; Ta33 <= FIXED_ONE;
                        state <= LOAD_DH;
                    end
                end

                LOAD_DH: begin
                    dh_d <= dh_d_w;
                    dh_a <= dh_a_w;
                    dh_alpha <= dh_alpha_w;
                    trig_phase <= 1'b0;
                    state <= CALC_TRIG;
                end

                CALC_TRIG: begin
                    if (!trig_phase) begin
                        cordic_angle <= q_arr[joint_idx];
                    end else begin
                        cordic_angle <= dh_alpha;
                    end
                    cordic_start <= 1'b1;
                    state <= WAIT_TRIG;
                end

                WAIT_TRIG: begin
                    cordic_start <= 1'b0;
                    if (cordic_done) begin
                        if (!trig_phase) begin
                            sin_q <= cordic_sin;
                            cos_q <= cordic_cos;
                            trig_phase <= 1'b1;
                            state <= CALC_TRIG;
                        end else begin
                            sin_alpha <= cordic_sin;
                            cos_alpha <= cordic_cos;
                            state <= CALC_Ti;
                        end
                    end
                end

                CALC_Ti: begin
                    // Build DH transformation matrix Ti
                    Ti00 <= cos_q;
                    Ti01 <= -fmult(sin_q, cos_alpha);
                    Ti02 <= fmult(sin_q, sin_alpha);
                    Ti03 <= fmult(dh_a, cos_q);

                    Ti10 <= sin_q;
                    Ti11 <= fmult(cos_q, cos_alpha);
                    Ti12 <= -fmult(cos_q, sin_alpha);
                    Ti13 <= fmult(dh_a, sin_q);

                    Ti20 <= FIXED_ZERO;
                    Ti21 <= sin_alpha;
                    Ti22 <= cos_alpha;
                    Ti23 <= dh_d;

                    Ti30 <= FIXED_ZERO;
                    Ti31 <= FIXED_ZERO;
                    Ti32 <= FIXED_ZERO;
                    Ti33 <= FIXED_ONE;

                    mult_step <= 4'd0;
                    state <= MULT_MATRIX;
                end

                // 流水化: 每行拆 MUL(寄存16积) + ADD(求和->Tt), 共 9 步(0..8)。
                // 数学逐项与原版一致, 仅把乘法与加法树分到两拍以收紧时序。
                MULT_MATRIX: begin
                    case (mult_step)
                        4'd0: begin // row0 MUL
                            pp0<=fmult(Ta00,Ti00); pp1<=fmult(Ta01,Ti10); pp2<=fmult(Ta02,Ti20); pp3<=fmult(Ta03,Ti30);
                            pp4<=fmult(Ta00,Ti01); pp5<=fmult(Ta01,Ti11); pp6<=fmult(Ta02,Ti21); pp7<=fmult(Ta03,Ti31);
                            pp8<=fmult(Ta00,Ti02); pp9<=fmult(Ta01,Ti12); pp10<=fmult(Ta02,Ti22); pp11<=fmult(Ta03,Ti32);
                            pp12<=fmult(Ta00,Ti03); pp13<=fmult(Ta01,Ti13); pp14<=fmult(Ta02,Ti23); pp15<=fmult(Ta03,Ti33);
                            mult_step <= 4'd1;
                        end
                        4'd1: begin // row0 ADD
                            Tt00<=pp0+pp1+pp2+pp3; Tt01<=pp4+pp5+pp6+pp7; Tt02<=pp8+pp9+pp10+pp11; Tt03<=pp12+pp13+pp14+pp15;
                            mult_step <= 4'd2;
                        end
                        4'd2: begin // row1 MUL
                            pp0<=fmult(Ta10,Ti00); pp1<=fmult(Ta11,Ti10); pp2<=fmult(Ta12,Ti20); pp3<=fmult(Ta13,Ti30);
                            pp4<=fmult(Ta10,Ti01); pp5<=fmult(Ta11,Ti11); pp6<=fmult(Ta12,Ti21); pp7<=fmult(Ta13,Ti31);
                            pp8<=fmult(Ta10,Ti02); pp9<=fmult(Ta11,Ti12); pp10<=fmult(Ta12,Ti22); pp11<=fmult(Ta13,Ti32);
                            pp12<=fmult(Ta10,Ti03); pp13<=fmult(Ta11,Ti13); pp14<=fmult(Ta12,Ti23); pp15<=fmult(Ta13,Ti33);
                            mult_step <= 4'd3;
                        end
                        4'd3: begin // row1 ADD
                            Tt10<=pp0+pp1+pp2+pp3; Tt11<=pp4+pp5+pp6+pp7; Tt12<=pp8+pp9+pp10+pp11; Tt13<=pp12+pp13+pp14+pp15;
                            mult_step <= 4'd4;
                        end
                        4'd4: begin // row2 MUL
                            pp0<=fmult(Ta20,Ti00); pp1<=fmult(Ta21,Ti10); pp2<=fmult(Ta22,Ti20); pp3<=fmult(Ta23,Ti30);
                            pp4<=fmult(Ta20,Ti01); pp5<=fmult(Ta21,Ti11); pp6<=fmult(Ta22,Ti21); pp7<=fmult(Ta23,Ti31);
                            pp8<=fmult(Ta20,Ti02); pp9<=fmult(Ta21,Ti12); pp10<=fmult(Ta22,Ti22); pp11<=fmult(Ta23,Ti32);
                            pp12<=fmult(Ta20,Ti03); pp13<=fmult(Ta21,Ti13); pp14<=fmult(Ta22,Ti23); pp15<=fmult(Ta23,Ti33);
                            mult_step <= 4'd5;
                        end
                        4'd5: begin // row2 ADD
                            Tt20<=pp0+pp1+pp2+pp3; Tt21<=pp4+pp5+pp6+pp7; Tt22<=pp8+pp9+pp10+pp11; Tt23<=pp12+pp13+pp14+pp15;
                            mult_step <= 4'd6;
                        end
                        4'd6: begin // row3 MUL
                            pp0<=fmult(Ta30,Ti00); pp1<=fmult(Ta31,Ti10); pp2<=fmult(Ta32,Ti20); pp3<=fmult(Ta33,Ti30);
                            pp4<=fmult(Ta30,Ti01); pp5<=fmult(Ta31,Ti11); pp6<=fmult(Ta32,Ti21); pp7<=fmult(Ta33,Ti31);
                            pp8<=fmult(Ta30,Ti02); pp9<=fmult(Ta31,Ti12); pp10<=fmult(Ta32,Ti22); pp11<=fmult(Ta33,Ti32);
                            pp12<=fmult(Ta30,Ti03); pp13<=fmult(Ta31,Ti13); pp14<=fmult(Ta32,Ti23); pp15<=fmult(Ta33,Ti33);
                            mult_step <= 4'd7;
                        end
                        4'd7: begin // row3 ADD
                            Tt30<=pp0+pp1+pp2+pp3; Tt31<=pp4+pp5+pp6+pp7; Tt32<=pp8+pp9+pp10+pp11; Tt33<=pp12+pp13+pp14+pp15;
                            mult_step <= 4'd8;
                        end
                        4'd8: begin // writeback Ta <= Tt
                            Ta00 <= Tt00; Ta01 <= Tt01; Ta02 <= Tt02; Ta03 <= Tt03;
                            Ta10 <= Tt10; Ta11 <= Tt11; Ta12 <= Tt12; Ta13 <= Tt13;
                            Ta20 <= Tt20; Ta21 <= Tt21; Ta22 <= Tt22; Ta23 <= Tt23;
                            Ta30 <= Tt30; Ta31 <= Tt31; Ta32 <= Tt32; Ta33 <= Tt33;
                            state <= NEXT_JOINT;
                        end
                        default: mult_step <= 4'd0;
                    endcase
                end

                NEXT_JOINT: begin
                    // Save frame data (z-axis = col2, origin = col3 of current Ta)
                    case (joint_idx)
                        3'd0: begin frame0_zx<=Ta02; frame0_zy<=Ta12; frame0_zz<=Ta22; frame0_px<=Ta03; frame0_py<=Ta13; frame0_pz<=Ta23; end
                        3'd1: begin frame1_zx<=Ta02; frame1_zy<=Ta12; frame1_zz<=Ta22; frame1_px<=Ta03; frame1_py<=Ta13; frame1_pz<=Ta23; end
                        3'd2: begin frame2_zx<=Ta02; frame2_zy<=Ta12; frame2_zz<=Ta22; frame2_px<=Ta03; frame2_py<=Ta13; frame2_pz<=Ta23; end
                        3'd3: begin frame3_zx<=Ta02; frame3_zy<=Ta12; frame3_zz<=Ta22; frame3_px<=Ta03; frame3_py<=Ta13; frame3_pz<=Ta23; end
                        3'd4: begin frame4_zx<=Ta02; frame4_zy<=Ta12; frame4_zz<=Ta22; frame4_px<=Ta03; frame4_py<=Ta13; frame4_pz<=Ta23; end
                        default: ;
                    endcase
                    if (joint_idx == 3'd5) begin
                        // Copy to output
                        T00 <= Ta00; T01 <= Ta01; T02 <= Ta02; T03 <= Ta03;
                        T10 <= Ta10; T11 <= Ta11; T12 <= Ta12; T13 <= Ta13;
                        T20 <= Ta20; T21 <= Ta21; T22 <= Ta22; T23 <= Ta23;
                        T30 <= Ta30; T31 <= Ta31; T32 <= Ta32; T33 <= Ta33;
                        state <= EXTRACT_POSE;
                    end else begin
                        joint_idx <= joint_idx + 1;
                        state <= LOAD_DH;
                    end
                end

                EXTRACT_POSE: begin
                    // Extract position directly
                    pose_x <= Ta03;
                    pose_y <= Ta13;
                    pose_z <= Ta23;
                    // Start atan2 for rz = atan2(r10, r00)
                    atan_x <= Ta00;
                    atan_y <= Ta10;
                    atan_start <= 1'b1;
                    state <= WAIT_ATAN1;
                end

                WAIT_ATAN1: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_rz <= atan_angle;
                        // atan2 for ry = atan2(-r20, sqrt(r00^2+r10^2))
                        // 补偿 CORDIC 矢量增益: magnitude = K*sqrt(.) (K~1.6468);
                        // 乘 1/K(=0.6073=39797 Q16.16) 还原真实 sqrt(.), 否则 ry 偏小~15%
                        atan_x <= fmult(atan_mag, 32'sd39797);
                        atan_y <= -Ta20;
                        atan_start <= 1'b1;
                        state <= WAIT_ATAN2;
                    end
                end

                WAIT_ATAN2: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_ry <= atan_angle;
                        // atan2 for rx = atan2(r21, r22)
                        atan_x <= Ta22;
                        atan_y <= Ta21;
                        atan_start <= 1'b1;
                        state <= WAIT_ATAN3;
                    end
                end

                WAIT_ATAN3: begin
                    atan_start <= 1'b0;
                    if (atan_done) begin
                        pose_rx <= atan_angle;
                        state <= DONE_ST;
                    end
                end

                DONE_ST: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
