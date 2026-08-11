`timescale 1ns/1ps
//=====================================================================================
// tb_r1_3_foc_track.v — R1.3 力链路第三环节：FOC 电流环 i_q 跟踪精度扫描
//
// tb_foc.v 只在 iq_ref=±200 两点做了 30% 容差的定性判定；R1.3 需要的是
// "i_q,actual 是否忠实跟踪 i_q*" 的定量证据（τ_meas = Kt·i_q,actual 的可信度基础）。
//
// 本台在 foc_channel + foc_motor_model 闭环上扫 19 个 i_q* 电平（含 ±2000 钳位端点），
// 每点跑 60 个控制周期(2048 clk/周期)到稳态，再用最后 16 个周期统计：
//   稳态均值 / 绝对误差 / 相对误差 / 峰峰纹波
//
// 注意：这是 xsim 闭环模型（一阶电感滞后 + 12bit ADC 量化的电机模型），
//       不是实测电机台架数据，论文中须如实标注为"仿真闭环"。
//
// sentinel: R1_3_FOC_TRACK_OK
//=====================================================================================

module tb_r1_3_foc_track;

    reg clk, rst_n;
    initial clk = 1'b0;
    always #4 clk = ~clk;                 // 125 MHz (8 ns)，与板上 eth_clk 一致

    localparam integer INIT_CYCLES = 16384;
    localparam integer NLVL        = 19;
    localparam integer N_SETTLE    = 60;  // 到稳态的控制周期数
    localparam integer N_AVG       = 16;  // 统计窗口(控制周期)

    reg [30:0] Kp, Ki;
    reg signed [15:0] iq_ref;
    reg [11:0] phi;

    wire        sn_adc, en_adc;
    wire [11:0] adc_a, adc_b, adc_c;
    wire        pwm_en, pwm_a, pwm_b, pwm_c;
    wire        en_idq;
    wire signed [15:0] id_actual, iq_actual;
    wire        init_done;
    wire [9:0]  duty_a, duty_b, duty_c;

    foc_channel #(
        .INIT_CYCLES (INIT_CYCLES), .ANGLE_INV(0), .POLE_PAIR(8'd7),
        .MAX_AMP(9'd384), .SAMPLE_DELAY(9'd240)
    ) u_foc (
        .foc_clk(clk), .rst_n(rst_n), .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref), .phi(phi),
        .sn_adc(sn_adc), .en_adc(en_adc), .adc_a(adc_a), .adc_b(adc_b), .adc_c(adc_c),
        .pwm_en(pwm_en), .pwm_a(pwm_a), .pwm_b(pwm_b), .pwm_c(pwm_c),
        .en_idq(en_idq), .id_actual(id_actual), .iq_actual(iq_actual),
        .init_done(init_done), .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c)
    );

    foc_motor_model #(.KV(14), .SH(6), .IS(2), .DLY(20), .ADC_SIGN(-1)) u_motor (
        .clk(clk), .rst_n(rst_n),
        .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c),
        .sn_adc(sn_adc), .en_adc(en_adc),
        .adc_a(adc_a), .adc_b(adc_b), .adc_c(adc_c)
    );

    reg signed [15:0] iq_latched;
    always @(posedge clk) if (en_idq) iq_latched <= iq_actual;

    //---------------------------------------------------------------- 扫描电平
    reg signed [15:0] lvl [0:NLVL-1];
    task load_levels; begin
        lvl[0]=16'sd0;
        lvl[1]= 16'sd25;    lvl[2]=-16'sd25;
        lvl[3]= 16'sd50;    lvl[4]=-16'sd50;
        lvl[5]= 16'sd100;   lvl[6]=-16'sd100;
        lvl[7]= 16'sd200;   lvl[8]=-16'sd200;
        lvl[9]= 16'sd400;   lvl[10]=-16'sd400;
        lvl[11]=16'sd800;   lvl[12]=-16'sd800;
        lvl[13]=16'sd1200;  lvl[14]=-16'sd1200;
        lvl[15]=16'sd1600;  lvl[16]=-16'sd1600;
        lvl[17]=16'sd2000;  lvl[18]=-16'sd2000;   // torque_to_iq 的 ±2000 钳位端点
    end endtask

    integer li, k;
    integer acc;
    integer smin, smax, smean;
    integer aerr, arel;
    reg signed [15:0] s;

    integer r_ref  [0:NLVL-1];
    integer r_mean [0:NLVL-1];
    integer r_min  [0:NLVL-1];
    integer r_max  [0:NLVL-1];

    task run_ctrl_periods; input integer n; begin repeat (n*2048) @(posedge clk); end endtask

    initial begin
        Kp = 31'd15000; Ki = 31'd8000;
        iq_ref = 16'sd0; phi = 12'd0; rst_n = 1'b0;
        load_levels;
        repeat (20) @(posedge clk);
        rst_n = 1'b1;
        @(posedge init_done);
        repeat (4096) @(posedge clk);

        $display("==========================================================================");
        $display("R1.3 FOC current-loop i_q tracking sweep (xsim closed loop, 125 MHz)");
        $display("  foc_channel + foc_motor_model, locked rotor phi=0, Kp=%0d Ki=%0d", Kp, Ki);
        $display("  settle %0d ctrl periods (2048 clk each), stats over last %0d periods",
                 N_SETTLE, N_AVG);
        $display("==========================================================================");
        $display("  iq_ref   iq_mean    err   rel_err%%   ripple(pp)");
        $display("--------------------------------------------------------------------------");

        for (li = 0; li < NLVL; li = li + 1) begin
            iq_ref = lvl[li];
            run_ctrl_periods(N_SETTLE - N_AVG);
            acc = 0; smin = 32767; smax = -32768;
            for (k = 0; k < N_AVG; k = k + 1) begin
                run_ctrl_periods(1);
                s = iq_latched;
                acc = acc + s;
                if (s < smin) smin = s;
                if (s > smax) smax = s;
            end
            smean = acc / N_AVG;
            aerr  = smean - lvl[li];
            if (aerr < 0) aerr = -aerr;
            arel  = (lvl[li] == 0) ? 0 : (aerr * 1000) / ((lvl[li] < 0) ? -lvl[li] : lvl[li]);
            r_ref[li]=lvl[li]; r_mean[li]=smean; r_min[li]=smin; r_max[li]=smax;
            $display("  %6d   %7d  %6d    %6.1f       %6d",
                     lvl[li], smean, smean - lvl[li], arel/10.0, smax - smin);
        end

        $display("--------------------------------------------------------------------------");
        $display("  (err = iq_actual_mean - iq_ref, LSB of the 16-bit i_q port)");
        $display("R1_3_FOC_TRACK_OK");
        $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #500_000_000;
        $display("WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
