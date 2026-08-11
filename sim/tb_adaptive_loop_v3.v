`timescale 1ns / 1ps
// ============================================================================
// tb_adaptive_loop_v3.v — adaptive_loop_v3 临时验证台 (M6.1+M6.2+M6.3 合一)
// ----------------------------------------------------------------------------
// 用单位雅可比 J (Jᵀ·F = F) 使 τ_cmd_i = Ks_i·Δx_i − Ds_i·ẋ_i, 便于推理.
// 五项验证:
//   T1 直通     : adaptive_enable=0 → Ks_out==Ks_nom, Ds_out==Ds_nom, τ_cmd 走标称增益
//   T2 先验载入 : prior_load 注入已知 K_env → K_env_est 读回相等
//   T3 单步+关系: enable=1 走一步, 反比律关系 Ks_out·K_env ≈ Ks_nom² (内部定义核对)
//   T4 多步收敛 : 恒定接触 N 步, K_env 始终有界 [K_ENV_MIN,K_ENV_MAX] 且变化逐步收窄
//   T5 死区冻结 : |q_err|<Q_DEADZONE → K_env 不变, Ks_out==Ks_nom
// 期望末尾打印 ALL TESTS PASSED.
// ============================================================================
module tb_adaptive_loop_v3;
    // -- Q16.16 常量 --
    localparam signed [31:0] Q_1_0    = 32'sd65536;
    localparam signed [31:0] Q_100    = 32'sd6553600;    // 100.0 (Ks_nom)
    localparam signed [31:0] Q_10     = 32'sd655360;     // 10.0  (Ds_nom)
    localparam signed [31:0] Q_0_01   = 32'sd655;        // 0.01  (Δx)
    localparam signed [31:0] Q_0_05   = 32'sd3277;       // 0.05  (q_err, > 死区)
    localparam signed [31:0] Q_0_0005 = 32'sd33;         // 0.0005(q_err, < 死区65)
    localparam signed [31:0] Q_1200   = 32'sd78643200;   // 1200.0
    localparam signed [31:0] Q_2000   = 32'sd131072000;  // 2000.0
    localparam signed [31:0] Q_P1000  = 32'sd65536000;   // 1000.0 (P 初值)
    // 模块内部 clamp 参数(与 DUT 默认一致, 仅供检查)
    localparam signed [31:0] KS_MIN   = 32'sd655360;     // 10.0
    localparam signed [31:0] KS_MAX   = 32'sd327680000;  // 5000.0
    localparam signed [31:0] K_ENV_MIN= 32'sd6554;       // 0.1
    localparam signed [31:0] K_ENV_MAX= 32'sd655360000;  // 10000.0

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

    integer errors; integer ci;

    adaptive_loop_v3 dut (
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

    // Q16.16 <-> real
    function real r_of; input signed [31:0] v; begin r_of = $itor(v) / 65536.0; end endfunction

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

    // 单位 J
    task load_identity_J;
        integer r,c;
        begin
            for (r=0;r<6;r=r+1) for (c=0;c<6;c=c+1)
                Jm[r*6+c] = (r==c) ? Q_1_0 : 32'sd0;
        end
    endtask

    // 载入 RLS 先验 (prior_load 高 2 拍)
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

    // 走一个控制步
    task run_step;
        begin
            @(posedge clk); start = 1'b1;
            @(posedge clk); start = 1'b0;
            wait (done == 1'b1);
            @(posedge clk);
        end
    endtask

    // ---- 检查辅助 ----
    task chk_eq32; // 期望相等 (Q16.16)
        input [127:0] name; input signed [31:0] got, exp;
        begin
            if (got !== exp) begin
                $display("  [FAIL] %0s: got %0d (%.4f) exp %0d (%.4f)", name, got, r_of(got), exp, r_of(exp));
                errors = errors + 1;
            end else
                $display("  [ ok ] %0s = %0d (%.4f)", name, got, r_of(got));
        end
    endtask

    task chk_rel; // 相对误差容差 (real)
        input [127:0] name; input real got, exp, tol;
        real e;
        begin
            e = (exp==0.0) ? (got-exp) : (got-exp)/exp;
            if (e < 0) e = -e;
            if (e > tol) begin
                $display("  [FAIL] %0s: got %.4f exp %.4f relerr %.4f > tol %.4f", name, got, exp, e, tol);
                errors = errors + 1;
            end else
                $display("  [ ok ] %0s: got %.4f ~ exp %.4f (relerr %.4f)", name, got, exp, e);
        end
    endtask

    task chk_bound; // 检查在 [lo,hi] 内
        input [127:0] name; input signed [31:0] v, lo, hi;
        begin
            if (v < lo || v > hi) begin
                $display("  [FAIL] %0s: %.4f out of [%.4f, %.4f]", name, r_of(v), r_of(lo), r_of(hi));
                errors = errors + 1;
            end else
                $display("  [ ok ] %0s: %.4f in [%.4f, %.4f]", name, r_of(v), r_of(lo), r_of(hi));
        end
    endtask

    real kprev, kcur, d_first, d_last;
    integer s;

    initial begin
        errors = 0;
        rst_n = 0; start = 0; prior_load = 0; adaptive_enable = 0;
        load_identity_J;
        set_uniform(Q_100, Q_10, Q_0_05, Q_0_01, 32'sd0, 32'sd0, 16'sd0);
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        // ================= T1: 直通 (adaptive_enable=0) =================
        $display("\n=== T1  passthrough (adaptive_enable=0) ===");
        adaptive_enable = 0;
        set_uniform(Q_100, Q_10, Q_0_05, Q_0_01, 32'sd0, 32'sd0, 16'sd0);
        run_step;
        // 自适应禁用 → Ks_out/Ds_out 直通标称
        chk_eq32("Ks_out_0", Ks_out_0, Q_100);
        chk_eq32("Ds_out_0", Ds_out_0, Q_10);
        chk_eq32("Ks_out_5", Ks_out_5, Q_100);
        // τ_cmd = Ks·Δx − Ds·ẋ = 100*0.01 − 0 = 1.0 Nm
        chk_rel("tau_cmd_0", r_of(tau_cmd_0), 1.0, 0.02);
        // torque_to_iq: iq = sat(fmult(τ,KT_INV), ±2000), 比较的是 Q16.16 原始值.
        // τ≈1.0 → fmult=20.0(Q16.16 原始=1,310,720) ≫ 2000 → 钉在 IQ_MAX=2000.
        // 这是 M5.1 已板验的既有行为; iq_ref 不参与自适应环(环用 iq_actual), 故不影响 RLS.
        chk_eq32("iq_ref_0 (sat IQ_MAX)", {{16{iq_ref_0[15]}},iq_ref_0}, 32'sd2000);

        // ================= T2: 先验载入 =================
        $display("\n=== T2  prior_load injection ===");
        load_prior(Q_2000, Q_P1000);   // K_env=2000, P=1000
        chk_eq32("K_env_est_0", K_env_est_0, Q_2000);
        chk_eq32("K_env_est_3", K_env_est_3, Q_2000);

        // ================= T3: 单步自适应 + 反比律关系核对 =================
        $display("\n=== T3  single-step adapt + inverse-law relation ===");
        load_prior(Q_1200, Q_P1000);   // 已知初值 K_env=1200, P=1000
        adaptive_enable = 1;
        // q_err=0.05(>死区), iq_actual=100 → τ_meas=100*0.05=5Nm, τ_cmd≈1Nm → τ_ext≈4Nm
        set_uniform(Q_100, Q_10, Q_0_05, Q_0_01, 32'sd0, 32'sd0, 16'sd100);
        run_step;
        $display("  K_env_est_0=%.3f  Ks_out_0=%.3f  Ds_out_0=%.3f  tau_ext_0=%.3f",
                 r_of(K_env_est_0), r_of(Ks_out_0), r_of(Ds_out_0), r_of(tau_ext_0));
        // K_env 应已被更新(≠1200)
        if (K_env_est_0 === Q_1200) begin
            $display("  [FAIL] K_env_0 未更新 (仍=1200)"); errors=errors+1;
        end else $display("  [ ok ] K_env_0 已更新: %.3f", r_of(K_env_est_0));
        chk_bound("K_env_0 bound", K_env_est_0, K_ENV_MIN, K_ENV_MAX);
        chk_bound("Ks_out_0 bound", Ks_out_0, KS_MIN, KS_MAX);
        // 反比律: 若 Ks_out 未触 clamp, 则 Ks_out·K_env ≈ Ks_nom² (=10000)
        if (Ks_out_0 > KS_MIN && Ks_out_0 < KS_MAX && K_env_est_0 > 0)
            chk_rel("Ks_out*K_env ~ Ks_nom^2", r_of(Ks_out_0)*r_of(K_env_est_0), 100.0*100.0, 0.05);
        // 阻尼比: Ds_out/Ds_nom ≈ Ks_out/Ks_nom
        if (Ks_out_0 > KS_MIN && Ks_out_0 < KS_MAX)
            chk_rel("Ds_out/Ds_nom ~ Ks_out/Ks_nom", r_of(Ds_out_0)/r_of(Q_10), r_of(Ks_out_0)/r_of(Q_100), 0.05);

        // ================= T4: 多步闭环收敛 =================
        $display("\n=== T4  multi-step closed-loop convergence (20 steps) ===");
        load_prior(Q_2000, Q_P1000);
        adaptive_enable = 1;
        set_uniform(Q_100, Q_10, Q_0_05, Q_0_01, 32'sd0, 32'sd0, 16'sd200); // 恒定接触
        kprev = 2000.0; d_first = 0.0; d_last = 0.0;
        for (s=0; s<20; s=s+1) begin
            run_step;
            kcur = r_of(K_env_est_0);
            $display("  step %2d: K_env_0=%9.3f  Ks_out_0=%9.3f  Ds_out_0=%8.3f  tau_cmd_0=%7.3f  tau_ext_0=%8.3f",
                     s, kcur, r_of(Ks_out_0), r_of(Ds_out_0), r_of(tau_cmd_0), r_of(tau_ext_0));
            // 每步都必须有界
            chk_bound("  K_env_0", K_env_est_0, K_ENV_MIN, K_ENV_MAX);
            if (s==0)  d_first = (kcur>kprev)?(kcur-kprev):(kprev-kcur);
            if (s==19) d_last  = (kcur>kprev)?(kcur-kprev):(kprev-kcur);
            kprev = kcur;
        end
        // 收敛: 末步变化应远小于首步变化 (RLS 增益随 P 衰减)
        $display("  |dK| first=%.4f  last=%.4f", d_first, d_last);
        if (d_last <= d_first) $display("  [ ok ] 变化收窄: last(%.4f) <= first(%.4f)", d_last, d_first);
        else begin $display("  [FAIL] 未收敛: last(%.4f) > first(%.4f)", d_last, d_first); errors=errors+1; end

        // ================= T5: 死区冻结 =================
        $display("\n=== T5  dead-zone freeze (|q_err|<Q_DEADZONE) ===");
        load_prior(Q_1200, Q_P1000);
        adaptive_enable = 1;
        set_uniform(Q_100, Q_10, Q_0_0005, Q_0_01, 32'sd0, 32'sd0, 16'sd200); // q_err 0.0005 < 死区
        run_step;
        chk_eq32("K_env_0 frozen", K_env_est_0, Q_1200);   // 死区 → K_env 不更新
        chk_eq32("Ks_out_0 (dz->nom)", Ks_out_0, Q_100);   // 死区 → 输出标称
        chk_eq32("Ds_out_0 (dz->nom)", Ds_out_0, Q_10);

        // ================= 汇总 =================
        $display("\n========================================");
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("TESTS FAILED: %0d error(s)", errors);
        $display("========================================");
        $finish;
    end

    // 看门狗
    initial begin
        #2000000;
        $display("TIMEOUT — simulation hung");
        $finish;
    end
endmodule
