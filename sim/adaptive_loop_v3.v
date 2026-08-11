`timescale 1ns / 1ps
// ============================================================================
// adaptive_loop_v3.v  —  临时验证模块：自适应阻抗 闭环 (M6.1+M6.2+M6.3 合一)
// ----------------------------------------------------------------------------
// 本模块把三件事编排成「一次 start 走完一个控制步」的单模块，纯仿真验证用：
//
//   M6.1  例化 adaptive_impedance_v2 跑逐关节 RLS 环境刚度估计 + 反比 Ks/Ds 自适应
//   M6.2  片上算外部力矩:  τ_meas = Kt · iq_actual ,  τ_ext = τ_meas − τ_cmd
//   M6.3  闭环: 自适应得到的 Ks_out/Ds_out 回灌 cart_impedance 的 K/D, 下一步生效
//
// 一个控制步的数据流 (start -> done):
//   1) cart_impedance(J, p=x_cur, xd=x_des, K=Ks_cur, D=Ds_cur, xv=ẋ) -> τ_cmd
//   2) torque_to_iq(τ_cmd)                                            -> iq_ref
//   3) τ_meas = Kt·iq_actual ;  τ_ext = τ_meas − τ_cmd                (片上)
//   4) adaptive_impedance_v2(τ_ext, q_err, Ks_nom, Ds_nom)            -> Ks_out/Ds_out/K_env
//   5) Ks_cur<=Ks_out, Ds_cur<=Ds_out  (闭环回灌, 下一步用), done<=1
//
// Kt 默认 0.05 Nm/A, 与 torque_to_iq 的 KT_INV=20 (=1/Kt) 互逆;
//   τ_meas(Q16.16) = iq_actual(整数计数) × KT_Q16 ,  KT_Q16 = 0.05·65536 = 3277.
// 若 iq_actual==iq_ref(FOC 理想跟踪) 则 τ_meas≈τ_cmd、τ_ext≈0 → 不自适应;
// 外部接触使 iq_actual≠iq_ref 时 τ_ext≠0 → RLS 辨识 K_env、Ks/Ds 自适应.
//
// adaptive_enable=0 时 adaptive 模块直通标称值 → Ks_cur=Ks_nom → 阻抗用标称增益.
// prior_load 高时只把 RLS 初值载入(由内部 adaptive 模块处理), 本编排 FSM 停在 IDLE,
//   并清 primed(下一步从标称增益起算). 主输入须在一个控制步内保持稳定(tb 满足).
//
// Q16.16 有符号定点. Verilog-2001. 不综合上板, 仅供 tb_adaptive_loop_v3 验证.
// ============================================================================
module adaptive_loop_v3 #(
    parameter signed [31:0] KT_Q16 = 32'sd3277   // 0.05 Nm/A 在 Q16.16 (=1/KT_INV)
) (
    input  wire clk,
    input  wire rst_n,
    input  wire start,

    // 雅可比 J (6x6 行优先, Q16.16) — 供 cart_impedance 的 Jᵀ·F
    input  wire signed [31:0] J00,J01,J02,J03,J04,J05,
    input  wire signed [31:0] J10,J11,J12,J13,J14,J15,
    input  wire signed [31:0] J20,J21,J22,J23,J24,J25,
    input  wire signed [31:0] J30,J31,J32,J33,J34,J35,
    input  wire signed [31:0] J40,J41,J42,J43,J44,J45,
    input  wire signed [31:0] J50,J51,J52,J53,J54,J55,

    input  wire signed [31:0] p0,p1,p2,p3,p4,p5,       // 当前位姿 x_cur
    input  wire signed [31:0] xd0,xd1,xd2,xd3,xd4,xd5, // 期望 x_des
    input  wire signed [31:0] xv0,xv1,xv2,xv3,xv4,xv5, // 笛卡尔速度 ẋ

    input  wire signed [31:0] Ks_nom_0,Ks_nom_1,Ks_nom_2,Ks_nom_3,Ks_nom_4,Ks_nom_5,
    input  wire signed [31:0] Ds_nom_0,Ds_nom_1,Ds_nom_2,Ds_nom_3,Ds_nom_4,Ds_nom_5,

    input  wire signed [31:0] q_err_0,q_err_1,q_err_2,q_err_3,q_err_4,q_err_5,

    input  wire signed [15:0] iq_actual_0,iq_actual_1,iq_actual_2,
                              iq_actual_3,iq_actual_4,iq_actual_5, // 实测电流(模拟 FOC 反馈)

    input  wire adaptive_enable,

    // 历史先验载入 (透传给内部 adaptive_impedance_v2)
    input  wire prior_load,
    input  wire signed [31:0] K_env_prior_0,K_env_prior_1,K_env_prior_2,
                              K_env_prior_3,K_env_prior_4,K_env_prior_5,
    input  wire signed [31:0] P_prior_0,P_prior_1,P_prior_2,
                              P_prior_3,P_prior_4,P_prior_5,

    // -- 输出 (监控) --------------------------------------------------------
    output reg  signed [31:0] tau_cmd_0,tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5,
    output reg  signed [15:0] iq_ref_0,iq_ref_1,iq_ref_2,iq_ref_3,iq_ref_4,iq_ref_5,
    output reg  signed [31:0] tau_ext_0,tau_ext_1,tau_ext_2,tau_ext_3,tau_ext_4,tau_ext_5,

    output wire signed [31:0] Ks_out_0,Ks_out_1,Ks_out_2,Ks_out_3,Ks_out_4,Ks_out_5,
    output wire signed [31:0] Ds_out_0,Ds_out_1,Ds_out_2,Ds_out_3,Ds_out_4,Ds_out_5,
    output wire signed [31:0] K_env_est_0,K_env_est_1,K_env_est_2,
                              K_env_est_3,K_env_est_4,K_env_est_5,

    output reg  done
);
    integer k;

    // ---- 闭环应用增益 (回灌) ----
    reg signed [31:0] Ks_cur [0:5];
    reg signed [31:0] Ds_cur [0:5];
    reg primed;   // 0=本步用标称增益(复位/先验后), 1=用上一步自适应增益

    // ---- 子模块握手 ----
    reg  imp_start, tiq_start, adapt_start;
    wire imp_done,  tiq_done,  adapt_done;

    wire signed [31:0] ct0,ct1,ct2,ct3,ct4,ct5;   // cart_impedance τ
    wire signed [15:0] tq0,tq1,tq2,tq3,tq4,tq5;    // torque_to_iq iq

    wire signed [31:0] aKs0,aKs1,aKs2,aKs3,aKs4,aKs5; // adaptive Ks_out
    wire signed [31:0] aDs0,aDs1,aDs2,aDs3,aDs4,aDs5; // adaptive Ds_out

    assign Ks_out_0=aKs0; assign Ks_out_1=aKs1; assign Ks_out_2=aKs2;
    assign Ks_out_3=aKs3; assign Ks_out_4=aKs4; assign Ks_out_5=aKs5;
    assign Ds_out_0=aDs0; assign Ds_out_1=aDs1; assign Ds_out_2=aDs2;
    assign Ds_out_3=aDs3; assign Ds_out_4=aDs4; assign Ds_out_5=aDs5;

    // -- 笛卡尔阻抗: τ_cmd = Jᵀ(K·Δx − D·ẋ), K/D 用回灌后的 Ks_cur/Ds_cur --
    cart_impedance imp (
        .clk(clk), .rst_n(rst_n), .start(imp_start),
        .J00(J00),.J01(J01),.J02(J02),.J03(J03),.J04(J04),.J05(J05),
        .J10(J10),.J11(J11),.J12(J12),.J13(J13),.J14(J14),.J15(J15),
        .J20(J20),.J21(J21),.J22(J22),.J23(J23),.J24(J24),.J25(J25),
        .J30(J30),.J31(J31),.J32(J32),.J33(J33),.J34(J34),.J35(J35),
        .J40(J40),.J41(J41),.J42(J42),.J43(J43),.J44(J44),.J45(J45),
        .J50(J50),.J51(J51),.J52(J52),.J53(J53),.J54(J54),.J55(J55),
        .p0(p0),.p1(p1),.p2(p2),.p3(p3),.p4(p4),.p5(p5),
        .xd0(xd0),.xd1(xd1),.xd2(xd2),.xd3(xd3),.xd4(xd4),.xd5(xd5),
        .K0(Ks_cur[0]),.K1(Ks_cur[1]),.K2(Ks_cur[2]),
        .K3(Ks_cur[3]),.K4(Ks_cur[4]),.K5(Ks_cur[5]),
        .D0(Ds_cur[0]),.D1(Ds_cur[1]),.D2(Ds_cur[2]),
        .D3(Ds_cur[3]),.D4(Ds_cur[4]),.D5(Ds_cur[5]),
        .xv0(xv0),.xv1(xv1),.xv2(xv2),.xv3(xv3),.xv4(xv4),.xv5(xv5),
        .tau0(ct0),.tau1(ct1),.tau2(ct2),.tau3(ct3),.tau4(ct4),.tau5(ct5),
        .done(imp_done)
    );

    // -- 力矩->电流: iq_ref = sat(τ_cmd·KT_INV) --
    torque_to_iq tiq (
        .clk(clk), .rst_n(rst_n), .start(tiq_start),
        .tau0(tau_cmd_0),.tau1(tau_cmd_1),.tau2(tau_cmd_2),
        .tau3(tau_cmd_3),.tau4(tau_cmd_4),.tau5(tau_cmd_5),
        .iq0(tq0),.iq1(tq1),.iq2(tq2),.iq3(tq3),.iq4(tq4),.iq5(tq5),
        .done(tiq_done)
    );

    // -- 自适应阻抗: τ_ext,q_err,Ks_nom,Ds_nom -> Ks_out/Ds_out/K_env_est --
    adaptive_impedance_v2 adapt (
        .clk(clk), .rst_n(rst_n), .start(adapt_start),
        .tau_ext_0(tau_ext_0),.tau_ext_1(tau_ext_1),.tau_ext_2(tau_ext_2),
        .tau_ext_3(tau_ext_3),.tau_ext_4(tau_ext_4),.tau_ext_5(tau_ext_5),
        .q_err_0(q_err_0),.q_err_1(q_err_1),.q_err_2(q_err_2),
        .q_err_3(q_err_3),.q_err_4(q_err_4),.q_err_5(q_err_5),
        .Ks_nom_0(Ks_nom_0),.Ks_nom_1(Ks_nom_1),.Ks_nom_2(Ks_nom_2),
        .Ks_nom_3(Ks_nom_3),.Ks_nom_4(Ks_nom_4),.Ks_nom_5(Ks_nom_5),
        .Ds_nom_0(Ds_nom_0),.Ds_nom_1(Ds_nom_1),.Ds_nom_2(Ds_nom_2),
        .Ds_nom_3(Ds_nom_3),.Ds_nom_4(Ds_nom_4),.Ds_nom_5(Ds_nom_5),
        .adaptive_enable(adaptive_enable),
        .prior_load(prior_load),
        .K_env_prior_0(K_env_prior_0),.K_env_prior_1(K_env_prior_1),.K_env_prior_2(K_env_prior_2),
        .K_env_prior_3(K_env_prior_3),.K_env_prior_4(K_env_prior_4),.K_env_prior_5(K_env_prior_5),
        .P_prior_0(P_prior_0),.P_prior_1(P_prior_1),.P_prior_2(P_prior_2),
        .P_prior_3(P_prior_3),.P_prior_4(P_prior_4),.P_prior_5(P_prior_5),
        .Ks_out_0(aKs0),.Ks_out_1(aKs1),.Ks_out_2(aKs2),
        .Ks_out_3(aKs3),.Ks_out_4(aKs4),.Ks_out_5(aKs5),
        .Ds_out_0(aDs0),.Ds_out_1(aDs1),.Ds_out_2(aDs2),
        .Ds_out_3(aDs3),.Ds_out_4(aDs4),.Ds_out_5(aDs5),
        .K_env_est_0(K_env_est_0),.K_env_est_1(K_env_est_1),.K_env_est_2(K_env_est_2),
        .K_env_est_3(K_env_est_3),.K_env_est_4(K_env_est_4),.K_env_est_5(K_env_est_5),
        .done(adapt_done)
    );

    // iq_actual 扁平化 + τ_meas 计算用
    wire signed [15:0] iqa [0:5];
    assign iqa[0]=iq_actual_0; assign iqa[1]=iq_actual_1; assign iqa[2]=iq_actual_2;
    assign iqa[3]=iq_actual_3; assign iqa[4]=iq_actual_4; assign iqa[5]=iq_actual_5;

    // τ_cmd 扁平视图(便于 τ_ext 计算)
    wire signed [31:0] ct [0:5];
    assign ct[0]=ct0; assign ct[1]=ct1; assign ct[2]=ct2;
    assign ct[3]=ct3; assign ct[4]=ct4; assign ct[5]=ct5;

    // ---- 编排 FSM ----
    localparam S_IDLE=3'd0, S_LOADK=3'd1, S_IMP=3'd2, S_TIQ=3'd3, S_ADAPT=3'd4, S_DONE=3'd5;
    reg [2:0] state;

    // τ_meas(Q16.16) = iq_actual(整数) × KT_Q16
    function signed [31:0] tau_meas;
        input signed [15:0] iq;
        reg signed [31:0] iq32;
        begin
            iq32     = {{16{iq[15]}}, iq};      // 符号扩展到 32 位
            tau_meas = iq32 * KT_Q16;            // 整数×Q16.16 = Q16.16
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE; done <= 1'b0;
            imp_start <= 0; tiq_start <= 0; adapt_start <= 0;
            primed <= 1'b0;
            for (k=0;k<6;k=k+1) begin Ks_cur[k] <= 0; Ds_cur[k] <= 0; end
            tau_cmd_0<=0;tau_cmd_1<=0;tau_cmd_2<=0;tau_cmd_3<=0;tau_cmd_4<=0;tau_cmd_5<=0;
            iq_ref_0<=0;iq_ref_1<=0;iq_ref_2<=0;iq_ref_3<=0;iq_ref_4<=0;iq_ref_5<=0;
            tau_ext_0<=0;tau_ext_1<=0;tau_ext_2<=0;tau_ext_3<=0;tau_ext_4<=0;tau_ext_5<=0;
        end else begin
            imp_start <= 0; tiq_start <= 0; adapt_start <= 0;

            if (prior_load) begin
                // 先验载入期间: 编排停在 IDLE, 清 primed(下一步从标称起算).
                // RLS 初值由内部 adaptive 模块在 prior_load 高时自行载入.
                state  <= S_IDLE;
                done   <= 1'b0;
                primed <= 1'b0;
            end else begin
                case (state)
                S_IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // 本步要用的 K/D: 未自适应过则取标称, 否则保持上一步回灌值
                        if (!primed) begin
                            Ks_cur[0]<=Ks_nom_0; Ks_cur[1]<=Ks_nom_1; Ks_cur[2]<=Ks_nom_2;
                            Ks_cur[3]<=Ks_nom_3; Ks_cur[4]<=Ks_nom_4; Ks_cur[5]<=Ks_nom_5;
                            Ds_cur[0]<=Ds_nom_0; Ds_cur[1]<=Ds_nom_1; Ds_cur[2]<=Ds_nom_2;
                            Ds_cur[3]<=Ds_nom_3; Ds_cur[4]<=Ds_nom_4; Ds_cur[5]<=Ds_nom_5;
                        end
                        state <= S_LOADK;
                    end
                end

                // Ks_cur/Ds_cur 已稳定, 启动笛卡尔阻抗
                S_LOADK: begin
                    imp_start <= 1'b1;
                    state <= S_IMP;
                end

                S_IMP: if (imp_done) begin
                    tau_cmd_0<=ct[0];tau_cmd_1<=ct[1];tau_cmd_2<=ct[2];
                    tau_cmd_3<=ct[3];tau_cmd_4<=ct[4];tau_cmd_5<=ct[5];
                    tiq_start <= 1'b1;        // τ_cmd 此拍寄存, 与 tiq_start 同沿生效, torque_to_iq 下沿采到有效值
                    state <= S_TIQ;
                end

                S_TIQ: if (tiq_done) begin
                    iq_ref_0<=tq0;iq_ref_1<=tq1;iq_ref_2<=tq2;
                    iq_ref_3<=tq3;iq_ref_4<=tq4;iq_ref_5<=tq5;
                    // 片上 τ_ext = Kt·iq_actual − τ_cmd
                    tau_ext_0 <= tau_meas(iqa[0]) - tau_cmd_0;
                    tau_ext_1 <= tau_meas(iqa[1]) - tau_cmd_1;
                    tau_ext_2 <= tau_meas(iqa[2]) - tau_cmd_2;
                    tau_ext_3 <= tau_meas(iqa[3]) - tau_cmd_3;
                    tau_ext_4 <= tau_meas(iqa[4]) - tau_cmd_4;
                    tau_ext_5 <= tau_meas(iqa[5]) - tau_cmd_5;
                    adapt_start <= 1'b1;
                    state <= S_ADAPT;
                end

                S_ADAPT: if (adapt_done) begin
                    // 闭环回灌: 自适应增益成为下一步的 K/D
                    Ks_cur[0]<=aKs0;Ks_cur[1]<=aKs1;Ks_cur[2]<=aKs2;
                    Ks_cur[3]<=aKs3;Ks_cur[4]<=aKs4;Ks_cur[5]<=aKs5;
                    Ds_cur[0]<=aDs0;Ds_cur[1]<=aDs1;Ds_cur[2]<=aDs2;
                    Ds_cur[3]<=aDs3;Ds_cur[4]<=aDs4;Ds_cur[5]<=aDs5;
                    primed <= 1'b1;
                    done <= 1'b1;
                    state <= S_DONE;
                end

                S_DONE: begin
                    done <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end
endmodule
