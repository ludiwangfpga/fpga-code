`timescale 1ns / 1ps
// tb_foc.v - FOC 电流环闭环仿真.
//   foc_channel(锁转子, phi 固定 -> psi=0) <-> foc_motor_model(电机+ADC 行为模型).
//   验证: 给 iq_ref 正/负阶跃, iq_actual 应收敛跟踪 iq_ref(同号 + 合理比例).
//
//   锁转子简化: phi 固定不变 -> init_phi=phi -> psi=N*(phi-init_phi)=0.
//   psi=0 时回路化简为标量: iq_ref -> PI -> Vq -> SVPWM -> duty -> 电机 -> 电流 -> ADC
//   -> Iabc -> Clarke -> Ibeta=Iq -> 回 PI.

module tb_foc;

    // ---- 时钟/复位 ----
    reg clk;
    reg rst_n;

    initial clk = 1'b0;
    always #4 clk = ~clk;          // 8ns 周期 = 125MHz

    // ---- DUT 参数(可调) ----
    localparam integer INIT_CYCLES = 16384;  // 缩短初始化(原 33554432 太长)
    localparam [7:0]   POLE_PAIR   = 8'd7;

    // PI 参数(可调)
    reg [30:0] Kp;
    reg [30:0] Ki;

    // 电机模型参数(可调)
    localparam integer M_KV   = 14;
    localparam integer M_SH   = 6;
    localparam integer M_IS   = 2;
    localparam integer M_DLY  = 20;
    localparam integer M_ASGN = -1;  // ADC 偏置符号: +1 或 -1 (符号问题靠这个翻); -1=负反馈

    // ---- 互联信号 ----
    reg  signed [15:0] iq_ref;
    reg         [11:0] phi;

    wire        sn_adc;
    wire        en_adc;
    wire [11:0] adc_a, adc_b, adc_c;

    wire        pwm_en, pwm_a, pwm_b, pwm_c;
    wire        en_idq;
    wire signed [15:0] id_actual, iq_actual;
    wire        init_done;
    wire [9:0]  duty_a, duty_b, duty_c;

    // ---- 实例化 FOC 通道 ----
    foc_channel #(
        .INIT_CYCLES (INIT_CYCLES),
        .ANGLE_INV   (0),
        .POLE_PAIR   (POLE_PAIR),
        .MAX_AMP     (9'd384),
        .SAMPLE_DELAY(9'd240)
    ) u_foc (
        .foc_clk   (clk),
        .rst_n     (rst_n),
        .Kp        (Kp),
        .Ki        (Ki),
        .iq_ref    (iq_ref),
        .phi       (phi),
        .sn_adc    (sn_adc),
        .en_adc    (en_adc),
        .adc_a     (adc_a),
        .adc_b     (adc_b),
        .adc_c     (adc_c),
        .pwm_en    (pwm_en),
        .pwm_a     (pwm_a),
        .pwm_b     (pwm_b),
        .pwm_c     (pwm_c),
        .en_idq    (en_idq),
        .id_actual (id_actual),
        .iq_actual (iq_actual),
        .init_done (init_done),
        .duty_a    (duty_a),
        .duty_b    (duty_b),
        .duty_c    (duty_c)
    );

    // ---- 实例化电机 + ADC 模型 ----
    foc_motor_model #(
        .KV       (M_KV),
        .SH       (M_SH),
        .IS       (M_IS),
        .DLY      (M_DLY),
        .ADC_SIGN (M_ASGN)
    ) u_motor (
        .clk     (clk),
        .rst_n   (rst_n),
        .duty_a  (duty_a),
        .duty_b  (duty_b),
        .duty_c  (duty_c),
        .sn_adc  (sn_adc),
        .en_adc  (en_adc),
        .adc_a   (adc_a),
        .adc_b   (adc_b),
        .adc_c   (adc_c)
    );

    // ---- 跟踪 iq_actual 最新值(在 en_idq 脉冲处锁存) ----
    reg signed [15:0] iq_latched;
    always @(posedge clk) begin
        if(en_idq) iq_latched <= iq_actual;
    end

    // ---- 测试控制 ----
    integer errors;
    integer k;
    reg signed [15:0] iq_meas;

    // 跑 n 个控制周期(每周期 2048 拍 = 16384ns), 期间不阻塞采样模型
    task run_ctrl_periods;
        input integer n;
        begin
            repeat (n * 2048) @(posedge clk);
        end
    endtask

    // 检查收敛: iq_latched 与 ref 同号, 且 |iq-ref| < |ref|*0.3 (定性)
    task check_track;
        input signed [15:0] ref;
        input [127:0] tag;
        reg signed [31:0] diff, tol, aref;
        begin
            iq_meas = iq_latched;
            aref = (ref < 0) ? -ref : ref;
            diff = iq_meas - ref;
            if(diff < 0) diff = -diff;
            tol  = (aref * 3) / 10;       // 30%
            $display("[%0s] iq_ref=%0d  iq_actual=%0d  |diff|=%0d  tol(30%%)=%0d",
                      tag, ref, iq_meas, diff, tol);
            // 同号检查(ref!=0 时)
            if(ref > 0 && iq_meas <= 0) begin
                $display("  -> FAIL: 符号错误(期望正, 实际<=0)"); errors = errors + 1;
            end else if(ref < 0 && iq_meas >= 0) begin
                $display("  -> FAIL: 符号错误(期望负, 实际>=0)"); errors = errors + 1;
            end else if(diff > tol) begin
                $display("  -> FAIL: 跟踪误差超 30%%"); errors = errors + 1;
            end else begin
                $display("  -> PASS");
            end
        end
    endtask

    initial begin
        errors  = 0;
        Kp      = 31'd15000;
        Ki      = 31'd8000;
        iq_ref  = 16'sd0;
        phi     = 12'd0;        // 锁转子: phi 固定
        rst_n   = 1'b0;

        repeat (20) @(posedge clk);
        rst_n = 1'b1;

        // 等初始化结束
        @(posedge init_done);
        $display("[INIT] init_done=1, phi=%0d (locked rotor, psi==0)", phi);
        repeat (4096) @(posedge clk);   // 让 init 后的状态机/PWM 稳定 2 个控制周期

        // ---- 正阶跃 ----
        iq_ref = 16'sd200;
        $display("\n=== STEP +200 ===");
        for(k = 0; k < 60; k = k + 1) begin
            run_ctrl_periods(1);
            if((k % 10) == 0 || k >= 55)
                $display("  period %2d: iq_actual=%0d", k, iq_latched);
        end
        check_track(16'sd200, "STEP+");

        // ---- 负阶跃 ----
        iq_ref = -16'sd200;
        $display("\n=== STEP -200 ===");
        for(k = 0; k < 60; k = k + 1) begin
            run_ctrl_periods(1);
            if((k % 10) == 0 || k >= 55)
                $display("  period %2d: iq_actual=%0d", k, iq_latched);
        end
        check_track(-16'sd200, "STEP-");

        // ---- 回零 ----
        iq_ref = 16'sd0;
        $display("\n=== STEP 0 ===");
        for(k = 0; k < 40; k = k + 1) begin
            run_ctrl_periods(1);
            if((k % 10) == 0 || k >= 35)
                $display("  period %2d: iq_actual=%0d", k, iq_latched);
        end

        $display("\n========================================");
        if(errors == 0) begin
            $display("FOC current loop converged. errors=0");
            $display("ALL TESTS PASSED");
        end else begin
            $display("FOC current loop FAILED. errors=%0d", errors);
        end
        $display("========================================");
        $finish;
    end

    // 看门狗: 防止永久挂死
    initial begin
        #200_000_000;   // 200ms 仿真时间上限
        $display("WATCHDOG TIMEOUT - simulation did not finish");
        $display("  last iq_actual=%0d, init_done=%0d", iq_latched, init_done);
        $finish;
    end

endmodule
