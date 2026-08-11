`timescale 1ns / 1ps

// analytical_jacobian_serial.v - Serial Jacobian Calculator (baseline)
//
// Uses fk_engine (serial) instead of fk_engine_ext (parallel).
// Jacobian columns pipelined: col0 in 1 cycle, cols1-5 in 3 cycles each (DP->MUL->SUB).
// Total latency: FK (~378 cycles) + J_pipeline(~16) + done(1) = ~395 cycles

module analytical_jacobian_serial (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] q0, q1, q2, q3, q4, q5,

    output wire signed [31:0] pose_x, pose_y, pose_z,
    output wire signed [31:0] pose_rx, pose_ry, pose_rz,

    output reg  signed [31:0] J00, J01, J02, J03, J04, J05,
    output reg  signed [31:0] J10, J11, J12, J13, J14, J15,
    output reg  signed [31:0] J20, J21, J22, J23, J24, J25,
    output reg  signed [31:0] J30, J31, J32, J33, J34, J35,
    output reg  signed [31:0] J40, J41, J42, J43, J44, J45,
    output reg  signed [31:0] J50, J51, J52, J53, J54, J55,

    output reg         done
);

    localparam S_IDLE    = 3'd0;
    localparam S_WAIT_FK = 3'd1;
    localparam S_CALC_DP = 3'd2;  // unused post-pipeline (S_WAIT_FK -> S_CALC_J directly)
    localparam S_CALC_J  = 3'd3;  // Pipelined: col0 1 cyc; cols1-5 3 cyc each (DP->MUL->SUB)
    localparam S_DONE    = 3'd4;

    reg [2:0] state;
    reg [2:0] col_idx;

    reg fk_start;
    wire fk_done;

    // Frame data from serial FK
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

    localparam signed [31:0] BASE_ZX = 32'sd0;
    localparam signed [31:0] BASE_ZY = 32'sd0;
    localparam signed [31:0] BASE_ZZ = 32'sd65536;

    // Serial FK engine (with frame output)
    fk_engine fk_inst (
        .clk(clk), .rst_n(rst_n), .start(fk_start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .T00(fk_T00), .T01(fk_T01), .T02(fk_T02), .T03(fk_T03),
        .T10(fk_T10), .T11(fk_T11), .T12(fk_T12), .T13(fk_T13),
        .T20(fk_T20), .T21(fk_T21), .T22(fk_T22), .T23(fk_T23),
        .T30(fk_T30), .T31(fk_T31), .T32(fk_T32), .T33(fk_T33),
        .pose_x(pose_x), .pose_y(pose_y), .pose_z(pose_z),
        .pose_rx(pose_rx), .pose_ry(pose_ry), .pose_rz(pose_rz),
        .frame0_zx(f0_zx), .frame0_zy(f0_zy), .frame0_zz(f0_zz),
        .frame1_zx(f1_zx), .frame1_zy(f1_zy), .frame1_zz(f1_zz),
        .frame2_zx(f2_zx), .frame2_zy(f2_zy), .frame2_zz(f2_zz),
        .frame3_zx(f3_zx), .frame3_zy(f3_zy), .frame3_zz(f3_zz),
        .frame4_zx(f4_zx), .frame4_zy(f4_zy), .frame4_zz(f4_zz),
        .frame0_px(f0_px), .frame0_py(f0_py), .frame0_pz(f0_pz),
        .frame1_px(f1_px), .frame1_py(f1_py), .frame1_pz(f1_pz),
        .frame2_px(f2_px), .frame2_py(f2_py), .frame2_pz(f2_pz),
        .frame3_px(f3_px), .frame3_py(f3_py), .frame3_pz(f3_pz),
        .frame4_px(f4_px), .frame4_py(f4_py), .frame4_pz(f4_pz),
        .done(fk_done)
    );

    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a, b;
        reg signed [63:0] temp;
        begin temp = $signed(a) * $signed(b); fmult = temp[47:16]; end
    endfunction

    // Difference vectors
    reg signed [31:0] dp_x, dp_y, dp_z;
    // Current frame z/p for column computation
    reg signed [31:0] cur_zx, cur_zy, cur_zz;
    // 流水化叉乘: 阶段相位 + 乘积寄存器 (MUL 拍寄存 6 个乘积, SUB 拍相减)
    reg [1:0] jphase;
    reg signed [31:0] pp0, pp1, pp2, pp3, pp4, pp5;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            done <= 0; state <= S_IDLE; fk_start <= 0; col_idx <= 0; jphase <= 0;
            J00<=0;J01<=0;J02<=0;J03<=0;J04<=0;J05<=0;
            J10<=0;J11<=0;J12<=0;J13<=0;J14<=0;J15<=0;
            J20<=0;J21<=0;J22<=0;J23<=0;J24<=0;J25<=0;
            J30<=0;J31<=0;J32<=0;J33<=0;J34<=0;J35<=0;
            J40<=0;J41<=0;J42<=0;J43<=0;J44<=0;J45<=0;
            J50<=0;J51<=0;J52<=0;J53<=0;J54<=0;J55<=0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin fk_start <= 1; state <= S_WAIT_FK; end
                end

                S_WAIT_FK: begin
                    fk_start <= 0;
                    if (fk_done) begin
                        col_idx <= 0; jphase <= 0;
                        state <= S_CALC_J;
                    end
                end

                // 流水化: col0 基座(1拍, 叉乘退化无乘法); col1..5 每列 3 拍 DP->MUL->SUB.
                // 把「dp 减 -> 32x32 乘 -> 减」拆三拍, 关键路径砍到 1 步, 收紧时序 (数学与原版逐项一致).
                S_CALC_J: begin
                    if (col_idx == 3'd0) begin
                        // z_{-1} = [0,0,1], dp = pe  => 叉乘 [-pe_y, pe_x, 0]
                        J00 <= -pe_y; J10 <= pe_x; J20 <= 0;
                        J30 <= BASE_ZX; J40 <= BASE_ZY; J50 <= BASE_ZZ;
                        col_idx <= 3'd1; jphase <= 2'd0;
                    end else begin
                        case (jphase)
                            2'd0: begin   // DP: 选本列坐标系, dp = pe - p, 锁存 cur_z
                                case (col_idx)
                                    3'd1: begin cur_zx<=f0_zx; cur_zy<=f0_zy; cur_zz<=f0_zz; dp_x<=pe_x-f0_px; dp_y<=pe_y-f0_py; dp_z<=pe_z-f0_pz; end
                                    3'd2: begin cur_zx<=f1_zx; cur_zy<=f1_zy; cur_zz<=f1_zz; dp_x<=pe_x-f1_px; dp_y<=pe_y-f1_py; dp_z<=pe_z-f1_pz; end
                                    3'd3: begin cur_zx<=f2_zx; cur_zy<=f2_zy; cur_zz<=f2_zz; dp_x<=pe_x-f2_px; dp_y<=pe_y-f2_py; dp_z<=pe_z-f2_pz; end
                                    3'd4: begin cur_zx<=f3_zx; cur_zy<=f3_zy; cur_zz<=f3_zz; dp_x<=pe_x-f3_px; dp_y<=pe_y-f3_py; dp_z<=pe_z-f3_pz; end
                                    3'd5: begin cur_zx<=f4_zx; cur_zy<=f4_zy; cur_zz<=f4_zz; dp_x<=pe_x-f4_px; dp_y<=pe_y-f4_py; dp_z<=pe_z-f4_pz; end
                                    default: ;
                                endcase
                                jphase <= 2'd1;
                            end
                            2'd1: begin   // MUL: 6 个乘积寄存 (单乘单独一拍)
                                pp0 <= fmult(cur_zy, dp_z); pp1 <= fmult(cur_zz, dp_y);
                                pp2 <= fmult(cur_zz, dp_x); pp3 <= fmult(cur_zx, dp_z);
                                pp4 <= fmult(cur_zx, dp_y); pp5 <= fmult(cur_zy, dp_x);
                                jphase <= 2'd2;
                            end
                            2'd2: begin   // SUB: J = 乘积差; 角速度行 = cur_z
                                case (col_idx)
                                    3'd1: begin J01<=pp0-pp1; J11<=pp2-pp3; J21<=pp4-pp5; J31<=cur_zx; J41<=cur_zy; J51<=cur_zz; end
                                    3'd2: begin J02<=pp0-pp1; J12<=pp2-pp3; J22<=pp4-pp5; J32<=cur_zx; J42<=cur_zy; J52<=cur_zz; end
                                    3'd3: begin J03<=pp0-pp1; J13<=pp2-pp3; J23<=pp4-pp5; J33<=cur_zx; J43<=cur_zy; J53<=cur_zz; end
                                    3'd4: begin J04<=pp0-pp1; J14<=pp2-pp3; J24<=pp4-pp5; J34<=cur_zx; J44<=cur_zy; J54<=cur_zz; end
                                    3'd5: begin J05<=pp0-pp1; J15<=pp2-pp3; J25<=pp4-pp5; J35<=cur_zx; J45<=cur_zy; J55<=cur_zz; end
                                    default: ;
                                endcase
                                if (col_idx == 3'd5) state <= S_DONE;
                                else begin col_idx <= col_idx + 1; jphase <= 2'd0; end
                            end
                            default: jphase <= 2'd0;
                        endcase
                    end
                end

                S_DONE: begin
                    done <= 1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
