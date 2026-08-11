`timescale 1ns / 1ps
// ============================================================================
// tb_r2_1_cycles.v — R2.1 拍数对照: adaptive_en=0/1 前向链 (cart_impedance+
//   torque_to_iq) 实测拍数. 10 组不同输入 × {en=0,en=1} × 各 1 步.
//
//   监听 adaptive_loop_v3 内部信号 (层次化引用):
//     uut.imp_start / uut.imp_done   : cart_impedance start->done
//     uut.tiq_start / uut.tiq_done   : torque_to_iq start->done
//     uut.state                       : 编排 FSM 状态 (S_LOADK=1, ..., S_DONE=5)
//
//   forward_total_cycles  = 从 state 进入 S_LOADK 起, 到 tiq_done 拉高的拍数
//   cart_imp_cycles       = imp_start 拉高后 1 拍 (S_IMP 起), 到 imp_done 的拍数
//   tiq_cycles            = tiq_start 拉高后 1 拍 (S_TIQ 起), 到 tiq_done 的拍数
//
//   R2.1 契约: en=0/1 时 forward_total_cycles 应严格相等(adaptive 仅旁路自适应
//   增益生成, 不影响 cart_impedance 与 torque_to_iq 的执行时序).
// ============================================================================
module tb_r2_1_cycles;
    // -- Q16.16 常量 --
    localparam signed [31:0] Q_1_0    = 32'sd65536;
    localparam signed [31:0] Q_100    = 32'sd6553600;    // 100.0 (Ks_nom)
    localparam signed [31:0] Q_10     = 32'sd655360;     // 10.0  (Ds_nom)
    localparam signed [31:0] Q_0_01   = 32'sd655;        // 0.01
    localparam signed [31:0] Q_0_05   = 32'sd3277;       // 0.05
    localparam signed [31:0] Q_P1000  = 32'sd65536000;   // 1000.0 (P 初值)
    localparam signed [31:0] Q_1200   = 32'sd78643200;   // 1200.0

    reg clk, rst_n, start;
    reg signed [31:0] Jm [0:35];
    reg signed [31:0] pp [0:5], xd [0:5], xv [0:5];
    reg signed [31:0] Ksn [0:5], Dsn [0:5], qer [0:5];
    reg signed [15:0] iqa [0:5];
    reg adaptive_enable, prior_load;
    reg signed [31:0] kpr [0:5], ppr [0:5];

    wire signed [31:0] tau_cmd_0,tau_cmd_1,tau_cmd_2,tau_cmd_3,tau_cmd_4,tau_cmd_5;
    wire signed [15:0] iq_ref_0,iq_ref_1,iq_ref_2,iq_ref_3,iq_ref_4,iq_ref_5;
    wire signed [31:0] tau_ext_0,tau_ext_1,tau_ext_2,tau_ext_3,tau_ext_4,tau_ext_5;
    wire signed [31:0] Ks_out_0,Ks_out_1,Ks_out_2,Ks_out_3,Ks_out_4,Ks_out_5;
    wire signed [31:0] Ds_out_0,Ds_out_1,Ds_out_2,Ds_out_3,Ds_out_4,Ds_out_5;
    wire signed [31:0] K_env_est_0,K_env_est_1,K_env_est_2,K_env_est_3,K_env_est_4,K_env_est_5;
    wire done;

    adaptive_loop_v3 uut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .J00(Jm[0]), .J01(Jm[1]), .J02(Jm[2]), .J03(Jm[3]), .J04(Jm[4]), .J05(Jm[5]),
        .J10(Jm[6]), .J11(Jm[7]), .J12(Jm[8]), .J13(Jm[9]), .J14(Jm[10]),.J15(Jm[11]),
        .J20(Jm[12]),.J21(Jm[13]),.J22(Jm[14]),.J23(Jm[15]),.J24(Jm[16]),.J25(Jm[17]),
        .J30(Jm[18]),.J31(Jm[19]),.J32(Jm[20]),.J33(Jm[21]),.J34(Jm[22]),.J35(Jm[23]),
        .J40(Jm[24]),.J41(Jm[25]),.J42(Jm[26]),.J43(Jm[27]),.J44(Jm[28]),.J45(Jm[29]),
        .J50(Jm[30]),.J51(Jm[31]),.J52(Jm[32]),.J53(Jm[33]),.J54(Jm[34]),.J55(Jm[35]),
        .p0(pp[0]),.p1(pp[1]),.p2(pp[2]),.p3(pp[3]),.p4(pp[4]),.p5(pp[5]),
        .xd0(xd[0]),.xd1(xd[1]),.xd2(xd[2]),.xd3(xd[3]),.xd4(xd[4]),.xd5(xd[5]),
        .xv0(xv[0]),.xv1(xv[1]),.xv2(xv[2]),.xv3(xv[3]),.xv4(xv[4]),.xv5(xv[5]),
        .Ks_nom_0(Ksn[0]),.Ks_nom_1(Ksn[1]),.Ks_nom_2(Ksn[2]),.Ks_nom_3(Ksn[3]),.Ks_nom_4(Ksn[4]),.Ks_nom_5(Ksn[5]),
        .Ds_nom_0(Dsn[0]),.Ds_nom_1(Dsn[1]),.Ds_nom_2(Dsn[2]),.Ds_nom_3(Dsn[3]),.Ds_nom_4(Dsn[4]),.Ds_nom_5(Dsn[5]),
        .q_err_0(qer[0]),.q_err_1(qer[1]),.q_err_2(qer[2]),.q_err_3(qer[3]),.q_err_4(qer[4]),.q_err_5(qer[5]),
        .iq_actual_0(iqa[0]),.iq_actual_1(iqa[1]),.iq_actual_2(iqa[2]),
        .iq_actual_3(iqa[3]),.iq_actual_4(iqa[4]),.iq_actual_5(iqa[5]),
        .adaptive_enable(adaptive_enable),
        .prior_load(prior_load),
        .K_env_prior_0(kpr[0]),.K_env_prior_1(kpr[1]),.K_env_prior_2(kpr[2]),
        .K_env_prior_3(kpr[3]),.K_env_prior_4(kpr[4]),.K_env_prior_5(kpr[5]),
        .P_prior_0(ppr[0]),.P_prior_1(ppr[1]),.P_prior_2(ppr[2]),
        .P_prior_3(ppr[3]),.P_prior_4(ppr[4]),.P_prior_5(ppr[5]),
        .tau_cmd_0(tau_cmd_0),.tau_cmd_1(tau_cmd_1),.tau_cmd_2(tau_cmd_2),
        .tau_cmd_3(tau_cmd_3),.tau_cmd_4(tau_cmd_4),.tau_cmd_5(tau_cmd_5),
        .iq_ref_0(iq_ref_0),.iq_ref_1(iq_ref_1),.iq_ref_2(iq_ref_2),
        .iq_ref_3(iq_ref_3),.iq_ref_4(iq_ref_4),.iq_ref_5(iq_ref_5),
        .tau_ext_0(tau_ext_0),.tau_ext_1(tau_ext_1),.tau_ext_2(tau_ext_2),
        .tau_ext_3(tau_ext_3),.tau_ext_4(tau_ext_4),.tau_ext_5(tau_ext_5),
        .Ks_out_0(Ks_out_0),.Ks_out_1(Ks_out_1),.Ks_out_2(Ks_out_2),
        .Ks_out_3(Ks_out_3),.Ks_out_4(Ks_out_4),.Ks_out_5(Ks_out_5),
        .Ds_out_0(Ds_out_0),.Ds_out_1(Ds_out_1),.Ds_out_2(Ds_out_2),
        .Ds_out_3(Ds_out_3),.Ds_out_4(Ds_out_4),.Ds_out_5(Ds_out_5),
        .K_env_est_0(K_env_est_0),.K_env_est_1(K_env_est_1),.K_env_est_2(K_env_est_2),
        .K_env_est_3(K_env_est_3),.K_env_est_4(K_env_est_4),.K_env_est_5(K_env_est_5),
        .done(done)
    );

    // 100MHz
    initial clk = 0;
    always #5 clk = ~clk;

    // 设 6 路同值标量输入
    task set_uniform;
        input signed [31:0] ks, ds, qe, x_des, x_cur, x_vel;
        input signed [15:0] iq;
        integer j;
        begin
            for (j=0;j<6;j=j+1) begin
                Ksn[j]=ks; Dsn[j]=ds; qer[j]=qe;
                xd[j]=x_des; pp[j]=x_cur; xv[j]=x_vel; iqa[j]=iq;
            end
        end
    endtask

    // 单位 J (Jᵀ·F = F, 便于推理)
    task load_identity_J;
        integer r,c;
        begin
            for (r=0;r<6;r=r+1) for (c=0;c<6;c=c+1)
                Jm[r*6+c] = (r==c) ? Q_1_0 : 32'sd0;
        end
    endtask

    // nontrivial J (来自 tb_impedance.v 的实际雅可比, 通用测试)
    task load_nontrivial_J;
        begin
            Jm[0]=-32'sd11578; Jm[1]=32'sd16922; Jm[2]=32'sd8895; Jm[3]=32'sd1802; Jm[4]=-32'sd808; Jm[5]=0;
            Jm[6]=32'sd20053; Jm[7]=32'sd9770; Jm[8]=32'sd5136; Jm[9]=-32'sd3121; Jm[10]=-32'sd466; Jm[11]=0;
            Jm[12]=0; Jm[13]=32'sd19551; Jm[14]=32'sd10283; Jm[15]=0; Jm[16]=32'sd3481; Jm[17]=0;
            Jm[18]=0; Jm[19]=32'sd32768; Jm[20]=32'sd32768; Jm[21]=32'sd14689; Jm[22]=32'sd32768; Jm[23]=32'sd54822;
            Jm[24]=0; Jm[25]=-32'sd56756; Jm[26]=-32'sd56756; Jm[27]=32'sd8481; Jm[28]=-32'sd56756; Jm[29]=32'sd31652;
            Jm[30]=32'sd65536; Jm[31]=0; Jm[32]=0; Jm[33]=-32'sd63303; Jm[34]=0; Jm[35]=32'sd16962;
        end
    endtask

    // 载入 RLS 先验
    task load_prior;
        input signed [31:0] kenv, pcov;
        integer j;
        begin
            for (j=0;j<6;j=j+1) begin kpr[j]=kenv; ppr[j]=pcov; end
            @(posedge clk); prior_load = 1'b1;
            @(posedge clk); @(posedge clk);
            prior_load = 1'b0;
            @(posedge clk);
        end
    endtask

    // -------- 拍数采样 ---------
    // 思路: 在边沿之间数 (posedge clk).
    // S_LOADK = 3'd1 (来自 adaptive_loop_v3), 我们等 uut.state==S_LOADK 那个时钟沿
    // 当 sample 周期开始: 从 cur_sample_active <= 1 起每个 posedge clk 加 1.
    // imp 拍数 = imp_start 拉高后到 imp_done 拉高之间(含)的 posedge 数
    // tiq 拍数 = tiq_start 拉高后到 tiq_done 拉高之间(含)的 posedge 数
    // forward_total = state 进入 S_LOADK (imp_start 拉高同拍)起到 tiq_done 那拍

    reg [15:0] imp_cnt, tiq_cnt, fwd_cnt;
    reg imp_active, tiq_active, fwd_active;
    reg imp_start_d, tiq_start_d, imp_done_d, tiq_done_d;
    reg [15:0] last_imp, last_tiq, last_fwd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imp_cnt<=0; tiq_cnt<=0; fwd_cnt<=0;
            imp_active<=0; tiq_active<=0; fwd_active<=0;
            imp_start_d<=0; tiq_start_d<=0; imp_done_d<=0; tiq_done_d<=0;
            last_imp<=0; last_tiq<=0; last_fwd<=0;
        end else begin
            imp_start_d <= uut.imp_start;
            tiq_start_d <= uut.tiq_start;

            // imp_start 上升沿(从下到上)开始计数
            if (uut.imp_start && !imp_start_d) begin
                imp_active <= 1; imp_cnt <= 1;
                // forward 链(S_LOADK→S_TIQ done) 与 imp_start 上沿同时启动
                fwd_active <= 1; fwd_cnt <= 1;
            end else if (imp_active) begin
                imp_cnt <= imp_cnt + 16'd1;
                if (uut.imp_done) begin
                    last_imp <= imp_cnt + 16'd1;
                    imp_active <= 0;
                end
            end

            // tiq_start 上升沿开始计数
            if (uut.tiq_start && !tiq_start_d) begin
                tiq_active <= 1; tiq_cnt <= 1;
            end else if (tiq_active) begin
                tiq_cnt <= tiq_cnt + 16'd1;
                if (uut.tiq_done) begin
                    last_tiq <= tiq_cnt + 16'd1;
                    tiq_active <= 0;
                end
            end

            // forward_total: 当 tiq_done 拉高, 停 fwd
            if (fwd_active) begin
                fwd_cnt <= fwd_cnt + 16'd1;
                if (uut.tiq_done) begin
                    last_fwd <= fwd_cnt + 16'd1;
                    fwd_active <= 0;
                end
            end
        end
    end

    // 走一个控制步
    task run_step;
        begin
            @(posedge clk); start = 1'b1;
            @(posedge clk); start = 1'b0;
            wait (done == 1'b1);
            @(posedge clk);
        end
    endtask

    // ---- 10 组测试输入 (Q16.16) ----
    integer tc, i, errors;
    reg [15:0] log_imp_en0  [0:9];
    reg [15:0] log_imp_en1  [0:9];
    reg [15:0] log_tiq_en0  [0:9];
    reg [15:0] log_tiq_en1  [0:9];
    reg [15:0] log_fwd_en0  [0:9];
    reg [15:0] log_fwd_en1  [0:9];

    // 把 6 个数值塞进 Ksn/Dsn/qer/xd/pp/xv/iqa
    task apply_inputs;
        input integer tcid;
        input signed [31:0] ks, ds, qe, x_des, x_cur, x_vel;
        input signed [15:0] iq;
        begin
            // 为了让 10 组输入有差异, 用 tcid 调制
            set_uniform(ks, ds, qe, x_des, x_cur, x_vel, iq);
            // 把每一路稍作偏移 (i*某常数), 增加非对称性
        end
    endtask

    integer t;

    // 跑一遍: 给定 J/输入与 en 设置, 触发一步, 抓回拍数到 log
    task do_one;
        input integer tcid;
        input adaptive_en;
        begin
            adaptive_enable = adaptive_en;
            // 复用上一次的输入信号, 仅切换 enable
            run_step;
            if (adaptive_en==0) begin
                log_imp_en0[tcid] = last_imp;
                log_tiq_en0[tcid] = last_tiq;
                log_fwd_en0[tcid] = last_fwd;
            end else begin
                log_imp_en1[tcid] = last_imp;
                log_tiq_en1[tcid] = last_tiq;
                log_fwd_en1[tcid] = last_fwd;
            end
            $display("  TC%0d en=%0d: imp_cycles=%0d tiq_cycles=%0d forward_total=%0d",
                     tcid, adaptive_en, last_imp, last_tiq, last_fwd);
        end
    endtask

    // 10 组输入: 不同 K, D, qe, x 偏置, iq
    task setup_tc;
        input integer i;
        integer j; reg signed [31:0] k, d, q_e, xc, xt, xv0; reg signed [15:0] iq0;
        begin
            // 用 i 调制(基于线性变化)
            k    = 32'sd6553600 + (i*32'sd655360);     // 100 + i*10
            d    = 32'sd655360  + (i*32'sd131072);     // 10 + i*2
            q_e  = 32'sd3277    + (i*32'sd1638);       // 0.05 + i*0.025
            xc   = (i-3) * 32'sd655;                   // -3*0.01 ... +6*0.01
            xt   = xc + 32'sd655;                      // x_des = x_cur + 0.01
            xv0  = (i & 1) ? 32'sd328 : -32'sd328;     // 速度交替
            iq0  = 16'sd50 + i*16'sd25;
            for (j=0;j<6;j=j+1) begin
                Ksn[j]=k; Dsn[j]=d; qer[j]=q_e;
                xd[j]=xt; pp[j]=xc; xv[j]=xv0; iqa[j]=iq0;
            end
            // 若 i 偶数用单位 J, 奇数用 nontrivial J (覆盖不同 J 形态)
            if (i & 1) load_nontrivial_J; else load_identity_J;
        end
    endtask

    initial begin
        errors = 0;
        rst_n = 0; start = 0; prior_load = 0; adaptive_enable = 0;
        // 默认 J 和参数
        load_identity_J;
        set_uniform(Q_100, Q_10, Q_0_05, Q_0_01, 32'sd0, 32'sd0, 16'sd0);
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        $display("====================================================");
        $display("R2.1 cycles: adaptive_en=0 vs adaptive_en=1, 10 inputs");
        $display("====================================================");

        // 注意: 自适应有持久状态(K_env, P, primed). 每组前先 load_prior 重置.
        for (t=0; t<10; t=t+1) begin
            // 复位自适应状态(prior_load 也会清 primed)
            load_prior(Q_1200, Q_P1000);
            // 设置本组输入
            setup_tc(t);
            // 跑 en=0
            do_one(t, 0);
            // 复位自适应状态再跑 en=1 (确保两组初态相同)
            load_prior(Q_1200, Q_P1000);
            setup_tc(t);
            do_one(t, 1);
        end

        // ---- 对照表 ----
        $display("");
        $display("=====================================================================");
        $display("R2.1 SUMMARY TABLE — forward_total_cycles  (cart_imp + tiq)");
        $display("=====================================================================");
        $display("  TC | en=0 imp  tiq  fwd  | en=1 imp  tiq  fwd  | fwd_eq?");
        $display("---------------------------------------------------------------------");
        for (i=0;i<10;i=i+1) begin
            $display("  T%0d |     %3d  %3d  %3d  |     %3d  %3d  %3d  | %s",
                     i,
                     log_imp_en0[i], log_tiq_en0[i], log_fwd_en0[i],
                     log_imp_en1[i], log_tiq_en1[i], log_fwd_en1[i],
                     (log_fwd_en0[i]===log_fwd_en1[i]) ? "EQ" : "NE");
            if (log_fwd_en0[i] !== log_fwd_en1[i]) errors=errors+1;
        end
        $display("=====================================================================");

        // sentinel 必须与 errors 挂钩: 曾经无条件打印 ALL TESTS PASSED, 会把失败run误判为通过.
        if (errors == 0) begin
            $display("R2_1_FORWARD_CYCLES_EQ_OK");
            $display("ALL TESTS PASSED");
        end else begin
            $display("R2_1_FORWARD_CYCLES_MISMATCH_FAIL: %0d cases differ", errors);
            $display("TESTS FAILED");
        end
        $finish;
    end

    initial begin
        #4000000;
        $display("TIMEOUT — simulation hung");
        $display("TESTS FAILED");
        $finish;
    end
endmodule
