// udp_fk_cmd.v — M5.3 FOC: 例化 torque_to_iq, op10 路径 S_IMP_WAIT->S_TIQ_WAIT->S_TX; op12 读 iq_ref; iq_out 端口.
//   op14 写 -> phi0..5 + adc(6电机×3相)(24×int16 LE,48B) -> 锁存喂 foc_array, 回 18B ACK
//   op13 读 -> 回 36B: 6电机×3相占空比(18×int16 LE, 各10位中心512)
//   op2 写 q0..q5(各4B LE) -> fk_start -> 等 fk_done -> 锁存 pose -> 回 18B ACK
//   op8 读 -> 回 24B: 字节0-11=位置(x,y,z LE); 字节12-23=欧拉角(rx,ry,rz LE)
//   op9 读 -> 回 144B = 6x6 雅可比 J (行优先 J00..J55, 各4B Q16.16 LE)
//   op10 写 x_des0..5 + K0..5 + D0..5 + ẋ0..5(各4B LE,共96B) -> 算 F=K·Δx−D·ẋ, τ=Jᵀ·F -> torque_to_iq -> 回 18B ACK
//   op11 读 -> 回 24B: τ0..τ5 (各4B Q16.16 LE)
//   op12 读 -> 回 24B: iq0..5(6×int32 LE, 16位iq符号扩展)
//   op15 写 enable(1B)+q_err0..5+Ks_nom0..5+Ds_nom0..5(各24B,共73B) -> 片上 τ_ext=Kt·iq_actual−τ_cmd,
//        跑 adaptive_impedance_v2(逐关节RLS环境刚度估计+反比 Ks/Ds 自适应), 锁存 Ks_out/Ds_out/K_env, 回 18B ACK
//        闭环: adaptive_en=1 时 cart_impedance 的 K/D 用 Ks_out/Ds_out(替换 op10 下发值); =0 时维持原行为
//   op16 写 K_env_prior0..5+P_prior0..5(各24B,共48B) -> prior_load 注入 RLS 初值, 回 18B ACK
//   op17 读 -> 回 72B: K_env_est0..5 + Ks_out0..5 + Ds_out0..5 (各4B Q16.16 LE)
//   op18 读 -> 回 24B: cyc_last/cyc_min/cyc_max/cyc_runs/魔数E2E2E2E2/0 (各4B LE) —— E2 片上周期统计(纯观察者)
//        窗口=R1.1 口径: op2 末字节锁存拍起(含 S_DECIDE 分发拍) -> fk_done; 暂停(网络间隔不计);
//        op10 末字节锁存拍续(含 S_DECIDE) -> imp_done -> tiq_done(iq_reg 写入 FOC 输入寄存器拍)止.
//        = 4 拍命令 FSM 锁存 + fk 422 + imp 119 + tiq 4 = 549 (论文 Table III)
//   op3: ACK   op1/7/其它: 不回   任一 error: 丢弃
module udp_fk_cmd (
    input             clk,
    input             rst,
    input      [7:0]  s_udp_axis_data,
    input             s_udp_axis_valid,
    input             s_udp_axis_last,
    input             s_udp_axis_error,
    output reg [7:0]  m_udp_axis_data,
    output reg        m_udp_axis_valid,
    output reg        m_udp_axis_last,
    input             m_udp_axis_ready,
    output reg [10:0] cfg_udp_len
    , output reg signed [15:0] iq_out0, iq_out1, iq_out2, iq_out3, iq_out4, iq_out5
    , input wire [9:0] duty_a_0, duty_b_0, duty_c_0
    , input wire [9:0] duty_a_1, duty_b_1, duty_c_1
    , input wire [9:0] duty_a_2, duty_b_2, duty_c_2
    , input wire [9:0] duty_a_3, duty_b_3, duty_c_3
    , input wire [9:0] duty_a_4, duty_b_4, duty_c_4
    , input wire [9:0] duty_a_5, duty_b_5, duty_c_5
    , output reg [11:0] phi_0, phi_1, phi_2, phi_3, phi_4, phi_5
    , output reg [11:0] adc_a_0, adc_b_0, adc_c_0, adc_a_1, adc_b_1, adc_c_1
    , output reg [11:0] adc_a_2, adc_b_2, adc_c_2, adc_a_3, adc_b_3, adc_c_3
    , output reg [11:0] adc_a_4, adc_b_4, adc_c_4, adc_a_5, adc_b_5, adc_c_5
    , input wire signed [15:0] iq_act_0, iq_act_1, iq_act_2, iq_act_3, iq_act_4, iq_act_5  // foc_array 实测 iq, 算 τ_ext
);
    localparam S_OP=4'd0, S_RX=4'd1, S_DECIDE=4'd2, S_FK_WAIT=4'd3, S_TX=4'd4, S_IMP_WAIT=4'd5, S_TIQ_WAIT=4'd6,
               S_ADAPT_WAIT=4'd7, S_PRIOR=4'd8;
    localparam [10:0] ACKLEN  = 11'd18;
    localparam [10:0] RDLEN   = 11'd24;
    localparam [10:0] JLEN    = 11'd144;         // op9 雅可比回复长度
    localparam [10:0] DUTYLEN = 11'd36;          // op13 PWM 占空比回复长度
    localparam [10:0] ADAPTLEN= 11'd72;          // op17 自适应输出回复长度
    localparam [7:0]  ACK_BYTE = 8'hAA;
    localparam [2:0]  TX_ACK = 3'd0, TX_POSE = 3'd1, TX_J = 3'd2, TX_TAU = 3'd3, TX_IQ = 3'd4, TX_DUTY = 3'd5, TX_ADAPT = 3'd6, TX_CYC = 3'd7;  // 回复源
    localparam [31:0] CYC_MAGIC = 32'hE2E2E2E2;  // op18 魔数: host 用于确认板上是含 E2 计数器的新 bit
    localparam signed [31:0] KT_Q16 = 32'sd3277; // Kt=0.05 Nm/A 在 Q16.16 (=1/KT_INV), 用于 τ_meas=Kt·iq_actual

    reg [3:0]  state;
    reg [15:0] idx;
    reg [7:0]  opcode;
    reg        pkt_err;
    reg [31:0] q_reg [0:5];      // 关节角 (Q16.16)
    reg [31:0] pose_reg [0:5];   // x,y,z,rx,ry,rz (Q16.16)
    reg [31:0] j_reg [0:35];     // op9 雅可比 36 值 (行优先)
    reg [31:0] xdes_reg [0:5];    // op10: x_des (M4.2)
    reg [31:0] K_reg    [0:5];    // op10: 对角刚度 K (M4.2)
    reg [31:0] D_reg    [0:5];    // op10: 对角阻尼 D (M4.3)
    reg [31:0] xv_reg   [0:5];    // op10: 笛卡尔速度 ẋ (M4.3)
    reg [31:0] tau_reg [0:5];     // 阻抗力矩结果 τ
    reg [7:0]  rbuf [0:23];      // op8 回复 24B
    reg [7:0]  tx_cnt;           // 加宽到 8 位 (容纳 144)
    reg [7:0]  tx_len;
    reg [2:0]  tx_src;           // TX_ACK / TX_POSE(rbuf) / TX_J / TX_TAU / TX_IQ
    reg signed [15:0] iq_reg [0:5];  // torque_to_iq 结果锁存
    reg [9:0] duty_reg [0:17];       // op13: 6电机×3相 PWM 占空比快照 (索引=3*电机+相, 相0=a/1=b/2=c)
    reg [15:0] phi16 [0:5];          // op14: 编码器 phi (低12位有效)
    reg [15:0] adc16 [0:17];         // op14: ADC 电流 (低12位有效), 6电机×3相
    // ---- 自适应阻抗 (op15/16/17) ----
    reg [31:0] qerr_reg [0:5];       // op15: 关节位置误差 q_err
    reg [31:0] ksnom_reg [0:5];      // op15: 标称刚度 Ks_nom
    reg [31:0] dsnom_reg [0:5];      // op15: 标称阻尼 Ds_nom
    reg [31:0] kprior_reg [0:5];     // op16: RLS 环境刚度先验 K_env_prior
    reg [31:0] pprior_reg [0:5];     // op16: RLS 协方差先验 P_prior
    reg [31:0] tauext_reg [0:5];     // 片上 τ_ext = Kt·iq_actual − τ_cmd
    reg signed [31:0] ksout_reg [0:5]; // 自适应输出 Ks_out (锁存)
    reg signed [31:0] dsout_reg [0:5]; // 自适应输出 Ds_out (锁存)
    reg signed [31:0] kenv_reg  [0:5]; // 估计环境刚度 K_env_est (锁存)
    reg        adaptive_en;          // 自适应使能 (op15 enable 字节); =1 时 cart_impedance K/D 用 Ks_out/Ds_out
    reg        prior_load_r;         // op16 prior_load 电平
    reg [1:0]  prior_cnt;            // prior_load 保持拍数
    // ---- E2 片上周期计数器 (op18, 纯观察者: 只读现有信号, 不驱动/不 gate 前向链任何信号) ----
    reg        cyc_run;              // 窗口开启 (op2 末字节锁存拍置 1; tiq_done 闭合或异常取消时清 0)
    reg        cyc_wait;             // =1: fk 段结束, 暂停计数, 等 op10 末字节 (host 网络间隔不计拍)
    reg [31:0] cyc_cnt;              // 当前窗口累计拍数
    reg [31:0] cyc_last;             // 最近一次完整前向链拍数 (期望恒 549)
    reg [31:0] cyc_min;              // 历史最小 (复位 32'hFFFFFFFF)
    reg [31:0] cyc_max;              // 历史最大
    reg [31:0] cyc_runs;             // 完整前向链完成次数

    // ---- FK 引擎 (低有效异步复位) ----
    wire fk_rst_n = ~rst;
    reg  fk_start;
    wire fk_done;
    wire signed [31:0] fk_x, fk_y, fk_z, fk_rx, fk_ry, fk_rz;

    wire signed [31:0] j_w [0:35];
    analytical_jacobian_serial jac_inst (
        .clk(clk), .rst_n(fk_rst_n), .start(fk_start),
        .q0(q_reg[0]), .q1(q_reg[1]), .q2(q_reg[2]),
        .q3(q_reg[3]), .q4(q_reg[4]), .q5(q_reg[5]),
        .pose_x(fk_x), .pose_y(fk_y), .pose_z(fk_z),
        .pose_rx(fk_rx), .pose_ry(fk_ry), .pose_rz(fk_rz),
        .J00(j_w[0]),  .J01(j_w[1]),  .J02(j_w[2]),  .J03(j_w[3]),  .J04(j_w[4]),  .J05(j_w[5]),
        .J10(j_w[6]),  .J11(j_w[7]),  .J12(j_w[8]),  .J13(j_w[9]),  .J14(j_w[10]), .J15(j_w[11]),
        .J20(j_w[12]), .J21(j_w[13]), .J22(j_w[14]), .J23(j_w[15]), .J24(j_w[16]), .J25(j_w[17]),
        .J30(j_w[18]), .J31(j_w[19]), .J32(j_w[20]), .J33(j_w[21]), .J34(j_w[22]), .J35(j_w[23]),
        .J40(j_w[24]), .J41(j_w[25]), .J42(j_w[26]), .J43(j_w[27]), .J44(j_w[28]), .J45(j_w[29]),
        .J50(j_w[30]), .J51(j_w[31]), .J52(j_w[32]), .J53(j_w[33]), .J54(j_w[34]), .J55(j_w[35]),
        .done(fk_done)
    );

    // ---- 闭环增益选择: 自适应使能时用 Ks_out/Ds_out, 否则用 op10 下发的 K/D ----
    wire signed [31:0] Kc [0:5];
    wire signed [31:0] Dc [0:5];
    genvar gk;
    generate for (gk=0; gk<6; gk=gk+1) begin : g_gainmux
        assign Kc[gk] = adaptive_en ? ksout_reg[gk] : $signed(K_reg[gk]);
        assign Dc[gk] = adaptive_en ? dsout_reg[gk] : $signed(D_reg[gk]);
    end endgenerate

    // ---- 阻抗引擎: F=K·Δx−D·ẋ -> τ=Jᵀ·F (M4.3 全量: 刚度+阻尼) ----
    reg  imp_start;
    wire imp_done;
    wire signed [31:0] tau_w [0:5];
    cart_impedance imp_inst (
        .clk(clk), .rst_n(fk_rst_n), .start(imp_start),
        .J00(j_reg[0]),  .J01(j_reg[1]),  .J02(j_reg[2]),  .J03(j_reg[3]),  .J04(j_reg[4]),  .J05(j_reg[5]),
        .J10(j_reg[6]),  .J11(j_reg[7]),  .J12(j_reg[8]),  .J13(j_reg[9]),  .J14(j_reg[10]), .J15(j_reg[11]),
        .J20(j_reg[12]), .J21(j_reg[13]), .J22(j_reg[14]), .J23(j_reg[15]), .J24(j_reg[16]), .J25(j_reg[17]),
        .J30(j_reg[18]), .J31(j_reg[19]), .J32(j_reg[20]), .J33(j_reg[21]), .J34(j_reg[22]), .J35(j_reg[23]),
        .J40(j_reg[24]), .J41(j_reg[25]), .J42(j_reg[26]), .J43(j_reg[27]), .J44(j_reg[28]), .J45(j_reg[29]),
        .J50(j_reg[30]), .J51(j_reg[31]), .J52(j_reg[32]), .J53(j_reg[33]), .J54(j_reg[34]), .J55(j_reg[35]),
        .p0(pose_reg[0]), .p1(pose_reg[1]), .p2(pose_reg[2]),
        .p3(pose_reg[3]), .p4(pose_reg[4]), .p5(pose_reg[5]),
        .xd0(xdes_reg[0]), .xd1(xdes_reg[1]), .xd2(xdes_reg[2]),
        .xd3(xdes_reg[3]), .xd4(xdes_reg[4]), .xd5(xdes_reg[5]),
        .K0(Kc[0]), .K1(Kc[1]), .K2(Kc[2]), .K3(Kc[3]), .K4(Kc[4]), .K5(Kc[5]),
        .D0(Dc[0]), .D1(Dc[1]), .D2(Dc[2]), .D3(Dc[3]), .D4(Dc[4]), .D5(Dc[5]),
        .xv0(xv_reg[0]), .xv1(xv_reg[1]), .xv2(xv_reg[2]),
        .xv3(xv_reg[3]), .xv4(xv_reg[4]), .xv5(xv_reg[5]),
        .tau0(tau_w[0]), .tau1(tau_w[1]), .tau2(tau_w[2]),
        .tau3(tau_w[3]), .tau4(tau_w[4]), .tau5(tau_w[5]),
        .done(imp_done)
    );

    // ---- 自适应阻抗 v2: 逐关节 RLS 环境刚度估计 + 反比 Ks/Ds 自适应 (op15/16/17) ----
    reg  adapt_start;
    wire adapt_done;
    wire signed [31:0] aKs_w [0:5], aDs_w [0:5], aKenv_w [0:5];
    adaptive_impedance_v2 adapt_inst (
        .clk(clk), .rst_n(fk_rst_n), .start(adapt_start),
        .tau_ext_0(tauext_reg[0]), .tau_ext_1(tauext_reg[1]), .tau_ext_2(tauext_reg[2]),
        .tau_ext_3(tauext_reg[3]), .tau_ext_4(tauext_reg[4]), .tau_ext_5(tauext_reg[5]),
        .q_err_0(qerr_reg[0]), .q_err_1(qerr_reg[1]), .q_err_2(qerr_reg[2]),
        .q_err_3(qerr_reg[3]), .q_err_4(qerr_reg[4]), .q_err_5(qerr_reg[5]),
        .Ks_nom_0(ksnom_reg[0]), .Ks_nom_1(ksnom_reg[1]), .Ks_nom_2(ksnom_reg[2]),
        .Ks_nom_3(ksnom_reg[3]), .Ks_nom_4(ksnom_reg[4]), .Ks_nom_5(ksnom_reg[5]),
        .Ds_nom_0(dsnom_reg[0]), .Ds_nom_1(dsnom_reg[1]), .Ds_nom_2(dsnom_reg[2]),
        .Ds_nom_3(dsnom_reg[3]), .Ds_nom_4(dsnom_reg[4]), .Ds_nom_5(dsnom_reg[5]),
        .adaptive_enable(adaptive_en),
        .prior_load(prior_load_r),
        .K_env_prior_0(kprior_reg[0]), .K_env_prior_1(kprior_reg[1]), .K_env_prior_2(kprior_reg[2]),
        .K_env_prior_3(kprior_reg[3]), .K_env_prior_4(kprior_reg[4]), .K_env_prior_5(kprior_reg[5]),
        .P_prior_0(pprior_reg[0]), .P_prior_1(pprior_reg[1]), .P_prior_2(pprior_reg[2]),
        .P_prior_3(pprior_reg[3]), .P_prior_4(pprior_reg[4]), .P_prior_5(pprior_reg[5]),
        .Ks_out_0(aKs_w[0]), .Ks_out_1(aKs_w[1]), .Ks_out_2(aKs_w[2]),
        .Ks_out_3(aKs_w[3]), .Ks_out_4(aKs_w[4]), .Ks_out_5(aKs_w[5]),
        .Ds_out_0(aDs_w[0]), .Ds_out_1(aDs_w[1]), .Ds_out_2(aDs_w[2]),
        .Ds_out_3(aDs_w[3]), .Ds_out_4(aDs_w[4]), .Ds_out_5(aDs_w[5]),
        .K_env_est_0(aKenv_w[0]), .K_env_est_1(aKenv_w[1]), .K_env_est_2(aKenv_w[2]),
        .K_env_est_3(aKenv_w[3]), .K_env_est_4(aKenv_w[4]), .K_env_est_5(aKenv_w[5]),
        .done(adapt_done)
    );

    // ---- torque_to_iq: τ → iq_ref (M5.1) ----
    reg  tiq_start;
    wire tiq_done;
    wire signed [15:0] iq_w0, iq_w1, iq_w2, iq_w3, iq_w4, iq_w5;
    torque_to_iq tiq_inst (
        .clk(clk), .rst_n(fk_rst_n), .start(tiq_start),
        .tau0(tau_reg[0]), .tau1(tau_reg[1]), .tau2(tau_reg[2]),
        .tau3(tau_reg[3]), .tau4(tau_reg[4]), .tau5(tau_reg[5]),
        .iq0(iq_w0), .iq1(iq_w1), .iq2(iq_w2),
        .iq3(iq_w3), .iq4(iq_w4), .iq5(iq_w5),
        .done(tiq_done)
    );

    function [7:0] j_byte;       // 第 c 字节(0..143): j_reg[c>>2] 的第 c[1:0] 字节(LE)
        input [7:0] c; reg [31:0] w; reg [5:0] vi;
        begin
            vi = (c[7:2] > 6'd35) ? 6'd35 : c[7:2];   // 防御性限幅: 有效 J 索引 0..35
            w = j_reg[vi];
            case (c[1:0])
                2'd0: j_byte = w[7:0];
                2'd1: j_byte = w[15:8];
                2'd2: j_byte = w[23:16];
                2'd3: j_byte = w[31:24];
            endcase
        end
    endfunction

    function [7:0] tau_byte;     // 第 c 字节(0..23): tau_reg[c>>2] 的第 c[1:0] 字节(LE)
        input [7:0] c; reg [31:0] w; reg [5:0] vi;
        begin
            vi = (c[7:2] > 6'd5) ? 6'd5 : c[7:2];   // 防御性限幅: 有效 τ 索引 0..5
            w = tau_reg[vi];
            case (c[1:0])
                2'd0: tau_byte = w[7:0];
                2'd1: tau_byte = w[15:8];
                2'd2: tau_byte = w[23:16];
                2'd3: tau_byte = w[31:24];
            endcase
        end
    endfunction

    function [7:0] iq_byte;        // 第 c 字节(0..23): iq_reg[c>>2] 符号扩展为 int32 的第 c[1:0] 字节(LE)
        input [7:0] c; reg signed [31:0] w; reg [5:0] vi;
        begin
            vi = (c[7:2] > 6'd5) ? 6'd5 : c[7:2];
            w = {{16{iq_reg[vi][15]}}, iq_reg[vi]};
            case (c[1:0])
                2'd0: iq_byte = w[7:0];
                2'd1: iq_byte = w[15:8];
                2'd2: iq_byte = w[23:16];
                default: iq_byte = w[31:24];
            endcase
        end
    endfunction

    function [7:0] duty_byte;       // 第 c 字节(0..35): duty_reg[c>>1] 的低/高字节(int16 LE)
        input [7:0] c; reg [9:0] d; reg [4:0] vi;
        begin
            vi = (c[7:1] > 5'd17) ? 5'd17 : c[7:1];   // 0..17
            d = duty_reg[vi];
            duty_byte = c[0] ? {6'd0, d[9:8]} : d[7:0];
        end
    endfunction

    function [7:0] adapt_byte;      // 第 c 字节(0..71): [0..23]=K_env_est [24..47]=Ks_out [48..71]=Ds_out (int32 LE)
        input [7:0] c; reg signed [31:0] w; reg [4:0] vi;
        begin
            vi = c[7:2];            // 0..17
            w = (vi < 5'd6)  ? kenv_reg[vi]
              : (vi < 5'd12) ? ksout_reg[vi-5'd6]
                             : dsout_reg[vi-5'd12];
            case (c[1:0])
                2'd0: adapt_byte = w[7:0];
                2'd1: adapt_byte = w[15:8];
                2'd2: adapt_byte = w[23:16];
                default: adapt_byte = w[31:24];
            endcase
        end
    endfunction

    function [7:0] cyc_byte;        // 第 c 字节(0..23): {last,min,max,runs,魔数,0} 的 LE 字节 (op18)
        input [7:0] c; reg [31:0] w; reg [5:0] vi;
        begin
            vi = (c[7:2] > 6'd5) ? 6'd5 : c[7:2];   // 防御性限幅: 有效字索引 0..5
            case (vi[2:0])
                3'd0: w = cyc_last;
                3'd1: w = cyc_min;
                3'd2: w = cyc_max;
                3'd3: w = cyc_runs;
                3'd4: w = CYC_MAGIC;
                default: w = 32'd0;
            endcase
            case (c[1:0])
                2'd0: cyc_byte = w[7:0];
                2'd1: cyc_byte = w[15:8];
                2'd2: cyc_byte = w[23:16];
                default: cyc_byte = w[31:24];
            endcase
        end
    endfunction

    function [7:0] tx_byte;
        input [7:0] c; input [2:0] src;
        begin
            case (src)
                TX_POSE: tx_byte = rbuf[c[4:0]];   // c 0..23
                TX_J:    tx_byte = j_byte(c);
                TX_TAU:  tx_byte = tau_byte(c);     // c 0..23
                TX_IQ:   tx_byte = iq_byte(c);      // c 0..23
                TX_DUTY: tx_byte = duty_byte(c);    // c 0..35
                TX_ADAPT:tx_byte = adapt_byte(c);   // c 0..71
                TX_CYC:  tx_byte = cyc_byte(c);     // c 0..23
                default: tx_byte = ACK_BYTE;
            endcase
        end
    endfunction

    // 把 op8 回复装进 rbuf: 位置真值 + 欧拉角真值
    task load_rbuf_pos; begin
        rbuf[0]<=pose_reg[0][7:0];  rbuf[1]<=pose_reg[0][15:8];  rbuf[2]<=pose_reg[0][23:16];  rbuf[3]<=pose_reg[0][31:24];
        rbuf[4]<=pose_reg[1][7:0];  rbuf[5]<=pose_reg[1][15:8];  rbuf[6]<=pose_reg[1][23:16];  rbuf[7]<=pose_reg[1][31:24];
        rbuf[8]<=pose_reg[2][7:0];  rbuf[9]<=pose_reg[2][15:8];  rbuf[10]<=pose_reg[2][23:16]; rbuf[11]<=pose_reg[2][31:24];
        rbuf[12]<=pose_reg[3][7:0]; rbuf[13]<=pose_reg[3][15:8]; rbuf[14]<=pose_reg[3][23:16]; rbuf[15]<=pose_reg[3][31:24];
        rbuf[16]<=pose_reg[4][7:0]; rbuf[17]<=pose_reg[4][15:8]; rbuf[18]<=pose_reg[4][23:16]; rbuf[19]<=pose_reg[4][31:24];
        rbuf[20]<=pose_reg[5][7:0]; rbuf[21]<=pose_reg[5][15:8]; rbuf[22]<=pose_reg[5][23:16]; rbuf[23]<=pose_reg[5][31:24];
    end endtask

    integer k;
    always @(posedge clk) begin
        if (rst) begin
            state<=S_OP; idx<=16'd0; opcode<=8'd0; pkt_err<=1'b0;
            tx_cnt<=8'd0; tx_len<=8'd18; tx_src<=TX_ACK; fk_start<=1'b0;
            m_udp_axis_data<=8'd0; m_udp_axis_valid<=1'b0; m_udp_axis_last<=1'b0;
            cfg_udp_len<=ACKLEN;
            for (k=0;k<6;k=k+1) begin q_reg[k]<=32'd0; pose_reg[k]<=32'd0; end
            for (k=0;k<24;k=k+1) rbuf[k]<=8'd0;
            for (k=0;k<36;k=k+1) j_reg[k]<=32'd0;
            for (k=0;k<6;k=k+1) begin
                xdes_reg[k]<=32'd0; K_reg[k]<=32'd0; D_reg[k]<=32'd0; xv_reg[k]<=32'd0; tau_reg[k]<=32'd0;
                iq_reg[k]<=16'sd0;
            end
            imp_start<=1'b0;
            tiq_start<=1'b0;
            for (k=0;k<18;k=k+1) duty_reg[k]<=10'd0;
            for (k=0;k<6;k=k+1)  phi16[k]<=16'd0;
            for (k=0;k<18;k=k+1) adc16[k]<=16'd0;
            for (k=0;k<6;k=k+1) begin
                qerr_reg[k]<=32'd0; ksnom_reg[k]<=32'd0; dsnom_reg[k]<=32'd0;
                kprior_reg[k]<=32'd0; pprior_reg[k]<=32'd0; tauext_reg[k]<=32'd0;
                ksout_reg[k]<=32'sd0; dsout_reg[k]<=32'sd0; kenv_reg[k]<=32'sd0;
            end
            adaptive_en<=1'b0; prior_load_r<=1'b0; prior_cnt<=2'd0; adapt_start<=1'b0;
        end else begin
            fk_start<=1'b0;   // 默认落低, 仅 S_DECIDE 拉高 1 拍
            imp_start<=1'b0;  // 默认落低, 仅 S_DECIDE(op10) 拉高 1 拍
            tiq_start<=1'b0;  // 默认落低, 仅 S_IMP_WAIT(imp_done) 拉高 1 拍
            adapt_start<=1'b0; // 默认落低, 仅 S_DECIDE(op15) 拉高 1 拍 (prior_load_r 不在此默认, 是电平)
            // duty_reg 每拍快照: 保持最新值供 op13 读取
            duty_reg[0]<=duty_a_0; duty_reg[1]<=duty_b_0; duty_reg[2]<=duty_c_0;
            duty_reg[3]<=duty_a_1; duty_reg[4]<=duty_b_1; duty_reg[5]<=duty_c_1;
            duty_reg[6]<=duty_a_2; duty_reg[7]<=duty_b_2; duty_reg[8]<=duty_c_2;
            duty_reg[9]<=duty_a_3; duty_reg[10]<=duty_b_3; duty_reg[11]<=duty_c_3;
            duty_reg[12]<=duty_a_4; duty_reg[13]<=duty_b_4; duty_reg[14]<=duty_c_4;
            duty_reg[15]<=duty_a_5; duty_reg[16]<=duty_b_5; duty_reg[17]<=duty_c_5;
            case (state)
            S_OP: begin
                m_udp_axis_valid<=1'b0; m_udp_axis_last<=1'b0;
                if (s_udp_axis_valid) begin
                    opcode<=s_udp_axis_data; idx<=16'd1; pkt_err<=s_udp_axis_error;
                    if (s_udp_axis_last) begin
                        if ((s_udp_axis_data==8'd3) && !s_udp_axis_error) begin
                            tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                            m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                            state<=S_TX;
                        end else state<=S_OP;
                    end else state<=S_RX;
                end
            end
            S_RX: begin
                if (s_udp_axis_valid) begin
                    pkt_err<=pkt_err | s_udp_axis_error;
                    idx<=idx+16'd1;
                    if (opcode==8'd2 && idx>=16'd7 && idx<=16'd30) begin : QCAP
                        reg [4:0] pos; reg [2:0] gi; reg [1:0] lane;
                        pos  = idx[4:0] - 5'd7;
                        gi   = pos[4:2];
                        lane = pos[1:0];
                        case (lane)
                            2'd0: q_reg[gi][7:0]   <= s_udp_axis_data;
                            2'd1: q_reg[gi][15:8]  <= s_udp_axis_data;
                            2'd2: q_reg[gi][23:16] <= s_udp_axis_data;
                            2'd3: q_reg[gi][31:24] <= s_udp_axis_data;
                        endcase
                    end
                    if (opcode==8'd10 && idx>=16'd7 && idx<=16'd102) begin : OP10CAP
                        reg [6:0] pos; reg [4:0] gi; reg [1:0] lane; reg [2:0] sub;
                        pos  = idx[7:0] - 8'd7;       // 0..95
                        gi   = pos[6:2];              // 0..23
                        lane = pos[1:0];
                        sub  = gi % 5'd6;             // 组内 0..5  (6/12/18 整除)
                        // gi: 0-5 xdes, 6-11 K, 12-17 D, 18-23 xv
                        case (lane)
                            2'd0: case (gi/5'd6)
                                3'd0: xdes_reg[sub][7:0]<=s_udp_axis_data;
                                3'd1: K_reg[sub][7:0]   <=s_udp_axis_data;
                                3'd2: D_reg[sub][7:0]   <=s_udp_axis_data;
                                default: xv_reg[sub][7:0]<=s_udp_axis_data; endcase
                            2'd1: case (gi/5'd6)
                                3'd0: xdes_reg[sub][15:8]<=s_udp_axis_data;
                                3'd1: K_reg[sub][15:8]   <=s_udp_axis_data;
                                3'd2: D_reg[sub][15:8]   <=s_udp_axis_data;
                                default: xv_reg[sub][15:8]<=s_udp_axis_data; endcase
                            2'd2: case (gi/5'd6)
                                3'd0: xdes_reg[sub][23:16]<=s_udp_axis_data;
                                3'd1: K_reg[sub][23:16]   <=s_udp_axis_data;
                                3'd2: D_reg[sub][23:16]   <=s_udp_axis_data;
                                default: xv_reg[sub][23:16]<=s_udp_axis_data; endcase
                            2'd3: case (gi/5'd6)
                                3'd0: xdes_reg[sub][31:24]<=s_udp_axis_data;
                                3'd1: K_reg[sub][31:24]   <=s_udp_axis_data;
                                3'd2: D_reg[sub][31:24]   <=s_udp_axis_data;
                                default: xv_reg[sub][31:24]<=s_udp_axis_data; endcase
                        endcase
                    end
                    if (opcode==8'd14 && idx>=16'd7 && idx<=16'd54) begin : OP14CAP
                        reg [5:0] pos; reg [4:0] vi; reg lane;
                        pos  = idx[5:0] - 6'd7;     // 0..47
                        vi   = pos[5:1];            // 0..23
                        lane = pos[0];              // 0=低字节 1=高字节
                        if (vi <= 5'd5) begin                       // phi
                            if (lane) phi16[vi[2:0]][15:8] <= s_udp_axis_data;
                            else      phi16[vi[2:0]][7:0]  <= s_udp_axis_data;
                        end else begin                              // adc: vi-6 -> 0..17
                            if (lane) adc16[vi-5'd6][15:8] <= s_udp_axis_data;
                            else      adc16[vi-5'd6][7:0]  <= s_udp_axis_data;
                        end
                    end
                    if (opcode==8'd15) begin
                        if (idx==16'd7) adaptive_en <= s_udp_axis_data[0];   // enable 字节
                        else if (idx>=16'd8 && idx<=16'd79) begin : OP15CAP
                            reg [6:0] pos; reg [4:0] gi; reg [1:0] lane; reg [2:0] sub; reg [2:0] grp;
                            pos  = idx[7:0] - 8'd8;     // 0..71
                            gi   = pos[6:2];            // 0..17
                            lane = pos[1:0];
                            grp  = gi/5'd6;             // 0=q_err 1=Ks_nom 2=Ds_nom
                            sub  = gi%5'd6;             // 组内 0..5
                            case (lane)
                                2'd0: case (grp) 3'd0: qerr_reg[sub][7:0]<=s_udp_axis_data;
                                                 3'd1: ksnom_reg[sub][7:0]<=s_udp_axis_data;
                                                 default: dsnom_reg[sub][7:0]<=s_udp_axis_data; endcase
                                2'd1: case (grp) 3'd0: qerr_reg[sub][15:8]<=s_udp_axis_data;
                                                 3'd1: ksnom_reg[sub][15:8]<=s_udp_axis_data;
                                                 default: dsnom_reg[sub][15:8]<=s_udp_axis_data; endcase
                                2'd2: case (grp) 3'd0: qerr_reg[sub][23:16]<=s_udp_axis_data;
                                                 3'd1: ksnom_reg[sub][23:16]<=s_udp_axis_data;
                                                 default: dsnom_reg[sub][23:16]<=s_udp_axis_data; endcase
                                2'd3: case (grp) 3'd0: qerr_reg[sub][31:24]<=s_udp_axis_data;
                                                 3'd1: ksnom_reg[sub][31:24]<=s_udp_axis_data;
                                                 default: dsnom_reg[sub][31:24]<=s_udp_axis_data; endcase
                            endcase
                        end
                    end
                    if (opcode==8'd16 && idx>=16'd7 && idx<=16'd54) begin : OP16CAP
                        reg [6:0] pos; reg [4:0] gi; reg [1:0] lane; reg [2:0] sub; reg grp;
                        pos  = idx[7:0] - 8'd7;     // 0..47
                        gi   = pos[6:2];            // 0..11
                        lane = pos[1:0];
                        grp  = (gi >= 5'd6);        // 0=K_env_prior 1=P_prior
                        sub  = gi%5'd6;             // 0..5
                        case (lane)
                            2'd0: if (!grp) kprior_reg[sub][7:0]<=s_udp_axis_data;   else pprior_reg[sub][7:0]<=s_udp_axis_data;
                            2'd1: if (!grp) kprior_reg[sub][15:8]<=s_udp_axis_data;  else pprior_reg[sub][15:8]<=s_udp_axis_data;
                            2'd2: if (!grp) kprior_reg[sub][23:16]<=s_udp_axis_data; else pprior_reg[sub][23:16]<=s_udp_axis_data;
                            2'd3: if (!grp) kprior_reg[sub][31:24]<=s_udp_axis_data; else pprior_reg[sub][31:24]<=s_udp_axis_data;
                        endcase
                    end
                    if (s_udp_axis_last) state<=S_DECIDE;
                end
            end
            S_DECIDE: begin
                m_udp_axis_valid<=1'b0; m_udp_axis_last<=1'b0;
                if (pkt_err) state<=S_OP;
                else case (opcode)
                    8'd2: begin fk_start<=1'b1; state<=S_FK_WAIT; end       // 写 -> 触发 FK
                    8'd8: begin                                            // 读 -> 回位置+哨兵
                        load_rbuf_pos;                                     // 装 rbuf[1..23](本拍非阻塞,下拍生效供 S_TX 用)
                        tx_src<=TX_POSE; tx_len<=8'd24; cfg_udp_len<=RDLEN; tx_cnt<=8'd0;
                        // 首字节(byte0)直接从 pose_reg 预载: 同拍 rbuf[0] 还未生效, 故走直读路径(=rbuf[0] 同值); byte1..23 由 S_TX 取 rbuf
                        m_udp_axis_data<=pose_reg[0][7:0]; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd9: begin                                            // 读雅可比 -> 回 144B
                        tx_src<=TX_J; tx_len<=8'd144; cfg_udp_len<=JLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=j_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd10: begin imp_start<=1'b1; state<=S_IMP_WAIT; end    // 写F+触发 -> 算τ
                    8'd11: begin                                            // 读 τ -> 24B
                        tx_src<=TX_TAU; tx_len<=8'd24; cfg_udp_len<=RDLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=tau_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd12: begin                                            // 读 iq_ref -> 24B
                        tx_src<=TX_IQ; tx_len<=8'd24; cfg_udp_len<=RDLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=iq_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd13: begin                                            // 读 PWM 占空比 -> 36B
                        tx_src<=TX_DUTY; tx_len<=8'd36; cfg_udp_len<=DUTYLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=duty_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd14: begin                                          // 写 phi+adc -> 仅锁存, 回 ACK
                        tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd15: begin                                          // 写自适应输入 -> 片上算 τ_ext, 跑 RLS 自适应
                        // τ_ext = Kt·iq_actual − τ_cmd(上次 op10 锁存的 tau_reg)
                        tauext_reg[0]<=($signed({{16{iq_act_0[15]}},iq_act_0})*KT_Q16) - $signed(tau_reg[0]);
                        tauext_reg[1]<=($signed({{16{iq_act_1[15]}},iq_act_1})*KT_Q16) - $signed(tau_reg[1]);
                        tauext_reg[2]<=($signed({{16{iq_act_2[15]}},iq_act_2})*KT_Q16) - $signed(tau_reg[2]);
                        tauext_reg[3]<=($signed({{16{iq_act_3[15]}},iq_act_3})*KT_Q16) - $signed(tau_reg[3]);
                        tauext_reg[4]<=($signed({{16{iq_act_4[15]}},iq_act_4})*KT_Q16) - $signed(tau_reg[4]);
                        tauext_reg[5]<=($signed({{16{iq_act_5[15]}},iq_act_5})*KT_Q16) - $signed(tau_reg[5]);
                        adapt_start<=1'b1;                                 // q_err/Ks_nom/Ds_nom/enable 已于 S_RX 捕获
                        state<=S_ADAPT_WAIT;
                    end
                    8'd16: begin                                          // 写 RLS 先验 -> prior_load 注入
                        prior_load_r<=1'b1; prior_cnt<=2'd0;
                        state<=S_PRIOR;
                    end
                    8'd17: begin                                          // 读自适应输出 -> 72B
                        tx_src<=TX_ADAPT; tx_len<=8'd72; cfg_udp_len<=ADAPTLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=adapt_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd18: begin                                          // 读 E2 周期统计 -> 24B (last/min/max/runs/魔数/0)
                        tx_src<=TX_CYC; tx_len<=8'd24; cfg_udp_len<=RDLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=cyc_byte(8'd0);
                        m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    8'd3: begin
                        tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                        m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                        state<=S_TX;
                    end
                    default: state<=S_OP;
                endcase
            end
            S_FK_WAIT: begin
                if (fk_done) begin
                    pose_reg[0]<=fk_x;  pose_reg[1]<=fk_y;  pose_reg[2]<=fk_z;
                    pose_reg[3]<=fk_rx; pose_reg[4]<=fk_ry; pose_reg[5]<=fk_rz;
                    j_reg[0]<=j_w[0];   j_reg[1]<=j_w[1];   j_reg[2]<=j_w[2];   j_reg[3]<=j_w[3];   j_reg[4]<=j_w[4];   j_reg[5]<=j_w[5];
                    j_reg[6]<=j_w[6];   j_reg[7]<=j_w[7];   j_reg[8]<=j_w[8];   j_reg[9]<=j_w[9];   j_reg[10]<=j_w[10]; j_reg[11]<=j_w[11];
                    j_reg[12]<=j_w[12]; j_reg[13]<=j_w[13]; j_reg[14]<=j_w[14]; j_reg[15]<=j_w[15]; j_reg[16]<=j_w[16]; j_reg[17]<=j_w[17];
                    j_reg[18]<=j_w[18]; j_reg[19]<=j_w[19]; j_reg[20]<=j_w[20]; j_reg[21]<=j_w[21]; j_reg[22]<=j_w[22]; j_reg[23]<=j_w[23];
                    j_reg[24]<=j_w[24]; j_reg[25]<=j_w[25]; j_reg[26]<=j_w[26]; j_reg[27]<=j_w[27]; j_reg[28]<=j_w[28]; j_reg[29]<=j_w[29];
                    j_reg[30]<=j_w[30]; j_reg[31]<=j_w[31]; j_reg[32]<=j_w[32]; j_reg[33]<=j_w[33]; j_reg[34]<=j_w[34]; j_reg[35]<=j_w[35];
                    tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                    m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                    state<=S_TX;
                end
            end
            S_IMP_WAIT: begin
                if (imp_done) begin
                    tau_reg[0]<=tau_w[0]; tau_reg[1]<=tau_w[1]; tau_reg[2]<=tau_w[2];
                    tau_reg[3]<=tau_w[3]; tau_reg[4]<=tau_w[4]; tau_reg[5]<=tau_w[5];
                    tiq_start<=1'b1;           // τ 已锁存(本拍 NB 写入), 下拍 torque_to_iq 读到新值
                    state<=S_TIQ_WAIT;
                end
            end
            S_TIQ_WAIT: begin
                if (tiq_done) begin
                    iq_reg[0]<=iq_w0; iq_reg[1]<=iq_w1; iq_reg[2]<=iq_w2;
                    iq_reg[3]<=iq_w3; iq_reg[4]<=iq_w4; iq_reg[5]<=iq_w5;
                    tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                    m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                    state<=S_TX;
                end
            end
            S_ADAPT_WAIT: begin
                if (adapt_done) begin
                    kenv_reg[0]<=aKenv_w[0]; kenv_reg[1]<=aKenv_w[1]; kenv_reg[2]<=aKenv_w[2];
                    kenv_reg[3]<=aKenv_w[3]; kenv_reg[4]<=aKenv_w[4]; kenv_reg[5]<=aKenv_w[5];
                    ksout_reg[0]<=aKs_w[0]; ksout_reg[1]<=aKs_w[1]; ksout_reg[2]<=aKs_w[2];
                    ksout_reg[3]<=aKs_w[3]; ksout_reg[4]<=aKs_w[4]; ksout_reg[5]<=aKs_w[5];
                    dsout_reg[0]<=aDs_w[0]; dsout_reg[1]<=aDs_w[1]; dsout_reg[2]<=aDs_w[2];
                    dsout_reg[3]<=aDs_w[3]; dsout_reg[4]<=aDs_w[4]; dsout_reg[5]<=aDs_w[5];
                    // 闭环回灌: Ks_out/Ds_out 经 Kc/Dc mux 自动成为下次 op10 的 K/D
                    tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                    m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                    state<=S_TX;
                end
            end
            S_PRIOR: begin
                // prior_load 高若干拍, 让 adaptive_impedance_v2 载入 K_env/P, 再回 ACK
                if (prior_cnt==2'd2) begin
                    prior_load_r<=1'b0;
                    // 把刚载入的 K_env 先验快照进 kenv_reg, 供 op17 直接读回(无需先跑 op15)
                    kenv_reg[0]<=aKenv_w[0]; kenv_reg[1]<=aKenv_w[1]; kenv_reg[2]<=aKenv_w[2];
                    kenv_reg[3]<=aKenv_w[3]; kenv_reg[4]<=aKenv_w[4]; kenv_reg[5]<=aKenv_w[5];
                    tx_src<=TX_ACK; tx_len<=8'd18; cfg_udp_len<=ACKLEN; tx_cnt<=8'd0;
                    m_udp_axis_data<=ACK_BYTE; m_udp_axis_last<=1'b0; m_udp_axis_valid<=1'b1;
                    state<=S_TX;
                end else prior_cnt<=prior_cnt+2'd1;
            end
            S_TX: begin
                m_udp_axis_valid<=1'b1;
                if (m_udp_axis_valid && m_udp_axis_ready) begin
                    if (tx_cnt==(tx_len-8'd1)) begin
                        m_udp_axis_valid<=1'b0; m_udp_axis_last<=1'b0; state<=S_OP;
                    end else begin
                        tx_cnt<=tx_cnt+8'd1;
                        m_udp_axis_data<=tx_byte(tx_cnt+8'd1, tx_src);
                        m_udp_axis_last<=((tx_cnt+8'd1)==(tx_len-8'd1));
                    end
                end
            end
            default: state<=S_OP;
            endcase
        end
    end

    // ================================================================
    // E2 片上周期计数器 (纯观察者): 独立 always 块, 只读命令 FSM/引擎的现有
    // 信号 (state/opcode/s_udp_axis_*/pkt_err/fk_done/tiq_done), 只写 cyc_* 新
    // 寄存器 —— 不驱动、不 gate 前向链的任何信号, 前向链行为零变化。
    //
    // R1.1 窗口口径 (= 论文 Table III 的 549 = 4 + 422 + 119 + 4):
    //   [开窗] op2 末字节被 FSM 接收的锁存拍 (state==S_RX && last && opcode==2), 计第 1 拍;
    //   [+1]   op2 的 S_DECIDE 分发拍 (fk_start 置位);
    //   [+422] S_FK_WAIT: fk_start 有效拍 -> fk_done 观察拍 (含);
    //   [暂停] fk 段结束 -> op10 到来之间 (回 ACK/网络间隔/中间读命令), 不计拍;
    //   [+1]   op10 末字节锁存拍;  [+1] op10 的 S_DECIDE 分发拍 (imp_start 置位);
    //   [+119] S_IMP_WAIT: imp_start 有效拍 -> imp_done 观察拍 (含);
    //   [+4]   S_TIQ_WAIT: tiq_start 有效拍 -> tiq_done 观察拍 (含) —— 该拍 iq_reg
    //          (FOC 输入寄存器) 非阻塞写入, 窗口闭合, 统计 last/min/max/runs。
    // 异常: 窗口内 S_DECIDE 遇 pkt_err 或非 op2/op10 分发 -> 取消本窗口 (不计 runs);
    //       暂停期再来 op2 -> 重开窗 (从头计)。
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            cyc_run<=1'b0; cyc_wait<=1'b0; cyc_cnt<=32'd0;
            cyc_last<=32'd0; cyc_min<=32'hFFFF_FFFF; cyc_max<=32'd0; cyc_runs<=32'd0;
        end else begin
            if (state==S_RX && s_udp_axis_valid && s_udp_axis_last && opcode==8'd2) begin
                cyc_run<=1'b1; cyc_wait<=1'b0; cyc_cnt<=32'd1;      // 开窗/重开窗: op2 锁存拍计 1
            end else if (cyc_run) begin
                if (!cyc_wait) begin
                    case (state)
                        S_DECIDE: begin                              // 分发拍 (op2->FK / op10->IMP)
                            if (!pkt_err && (opcode==8'd2 || opcode==8'd10)) cyc_cnt<=cyc_cnt+32'd1;
                            else cyc_run<=1'b0;                      // 错包/其它分支: 取消本窗口
                        end
                        S_FK_WAIT: begin
                            cyc_cnt<=cyc_cnt+32'd1;
                            if (fk_done) cyc_wait<=1'b1;             // fk 段完 -> 暂停, 等 op10
                        end
                        S_IMP_WAIT: cyc_cnt<=cyc_cnt+32'd1;
                        S_TIQ_WAIT: begin
                            cyc_cnt<=cyc_cnt+32'd1;
                            if (tiq_done) begin                      // iq_reg 写入拍: 窗口闭合
                                cyc_last<=cyc_cnt+32'd1;
                                if (cyc_cnt+32'd1 < cyc_min) cyc_min<=cyc_cnt+32'd1;
                                if (cyc_cnt+32'd1 > cyc_max) cyc_max<=cyc_cnt+32'd1;
                                cyc_runs<=cyc_runs+32'd1;
                                cyc_run<=1'b0; cyc_wait<=1'b0;
                            end
                        end
                        default: ;                                   // 防御: 其它状态不计
                    endcase
                end else begin
                    // 暂停期: 只等 op10 末字节锁存拍恢复计数 (op2 重开由上面分支处理)
                    if (state==S_RX && s_udp_axis_valid && s_udp_axis_last && opcode==8'd10) begin
                        cyc_wait<=1'b0; cyc_cnt<=cyc_cnt+32'd1;     // op10 锁存拍计 1
                    end
                end
            end
        end
    end

    // iq_out 持续驱动 (组合)
    always @(*) begin
        iq_out0=iq_reg[0]; iq_out1=iq_reg[1]; iq_out2=iq_reg[2];
        iq_out3=iq_reg[3]; iq_out4=iq_reg[4]; iq_out5=iq_reg[5];
    end

    // phi/adc 组合输出 (op14 锁存值持续驱动)
    always @(*) begin
        phi_0=phi16[0][11:0]; phi_1=phi16[1][11:0]; phi_2=phi16[2][11:0];
        phi_3=phi16[3][11:0]; phi_4=phi16[4][11:0]; phi_5=phi16[5][11:0];
        adc_a_0=adc16[0][11:0];  adc_b_0=adc16[1][11:0];  adc_c_0=adc16[2][11:0];
        adc_a_1=adc16[3][11:0];  adc_b_1=adc16[4][11:0];  adc_c_1=adc16[5][11:0];
        adc_a_2=adc16[6][11:0];  adc_b_2=adc16[7][11:0];  adc_c_2=adc16[8][11:0];
        adc_a_3=adc16[9][11:0];  adc_b_3=adc16[10][11:0]; adc_c_3=adc16[11][11:0];
        adc_a_4=adc16[12][11:0]; adc_b_4=adc16[13][11:0]; adc_c_4=adc16[14][11:0];
        adc_a_5=adc16[15][11:0]; adc_b_5=adc16[16][11:0]; adc_c_5=adc16[17][11:0];
    end
endmodule
