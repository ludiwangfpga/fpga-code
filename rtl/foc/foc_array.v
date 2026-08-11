`timescale 1ns / 1ps

// foc_array.v - 6-Channel FOC Motor Controller Array
//
// Instantiates 6 independent FOC channels for 6-DOF robot arm.
// Each channel has its own encoder, ADC, and PWM interface.
// All channels share the same FOC clock and PI parameters (can be overridden per-joint).
//
// Integration:
//   torque_to_iq.v (sys_clk) → iq_ref[0:5] → foc_array (foc_clk) → PWM[0:5]
//   encoder[0:5] → phi[0:5] → foc_array + encoder_interface → angle_rad[0:5] (for FK)

module foc_array #(
    // FOC initialization time (in FOC clock cycles)
    parameter INIT_CYCLES = 33554432,

    // Per-joint motor parameters
    parameter [7:0] POLE_PAIR_0 = 8'd7,
    parameter [7:0] POLE_PAIR_1 = 8'd7,
    parameter [7:0] POLE_PAIR_2 = 8'd7,
    parameter [7:0] POLE_PAIR_3 = 8'd7,
    parameter [7:0] POLE_PAIR_4 = 8'd7,
    parameter [7:0] POLE_PAIR_5 = 8'd7,

    // Per-joint angle sensor inversion
    parameter ANGLE_INV_0 = 0,
    parameter ANGLE_INV_1 = 0,
    parameter ANGLE_INV_2 = 0,
    parameter ANGLE_INV_3 = 0,
    parameter ANGLE_INV_4 = 0,
    parameter ANGLE_INV_5 = 0,

    // SVPWM parameters (shared)
    parameter [8:0] MAX_AMP      = 9'd384,
    parameter [8:0] SAMPLE_DELAY = 9'd240
) (
    // Clocks and reset
    input  wire        foc_clk,     // FOC clock (~73.728MHz)
    input  wire        rst_n,       // Active-low reset

    // PI parameters (shared across all channels, can be muxed externally if needed)
    input  wire [30:0] Kp,
    input  wire [30:0] Ki,

    // Iq references from torque_to_iq (signed 16-bit, system clock domain)
    input  wire signed [15:0] iq_ref_0, iq_ref_1, iq_ref_2,
    input  wire signed [15:0] iq_ref_3, iq_ref_4, iq_ref_5,

    // Encoder inputs (12-bit mechanical angle, 0-4095)
    input  wire [11:0] phi_0, phi_1, phi_2,
    input  wire [11:0] phi_3, phi_4, phi_5,

    // ADC interfaces (directly to/from external ADCs)
    // Channel 0
    output wire        sn_adc_0,
    input  wire        en_adc_0,
    input  wire [11:0] adc_a_0, adc_b_0, adc_c_0,
    // Channel 1
    output wire        sn_adc_1,
    input  wire        en_adc_1,
    input  wire [11:0] adc_a_1, adc_b_1, adc_c_1,
    // Channel 2
    output wire        sn_adc_2,
    input  wire        en_adc_2,
    input  wire [11:0] adc_a_2, adc_b_2, adc_c_2,
    // Channel 3
    output wire        sn_adc_3,
    input  wire        en_adc_3,
    input  wire [11:0] adc_a_3, adc_b_3, adc_c_3,
    // Channel 4
    output wire        sn_adc_4,
    input  wire        en_adc_4,
    input  wire [11:0] adc_a_4, adc_b_4, adc_c_4,
    // Channel 5
    output wire        sn_adc_5,
    input  wire        en_adc_5,
    input  wire [11:0] adc_a_5, adc_b_5, adc_c_5,

    // PWM outputs (active-high, active when pwm_en=1)
    output wire        pwm_en_0, pwm_a_0, pwm_b_0, pwm_c_0,
    output wire        pwm_en_1, pwm_a_1, pwm_b_1, pwm_c_1,
    output wire        pwm_en_2, pwm_a_2, pwm_b_2, pwm_c_2,
    output wire        pwm_en_3, pwm_a_3, pwm_b_3, pwm_c_3,
    output wire        pwm_en_4, pwm_a_4, pwm_b_4, pwm_c_4,
    output wire        pwm_en_5, pwm_a_5, pwm_b_5, pwm_c_5,

    // Current feedback (FOC clock domain)
    output wire signed [15:0] iq_actual_0, iq_actual_1, iq_actual_2,
    output wire signed [15:0] iq_actual_3, iq_actual_4, iq_actual_5,

    // All channels initialized
    output wire        all_init_done,

    // SVPWM 三相占空比抽头（6通道 × 3相，中心512）
    output wire [9:0] duty_a_0, duty_b_0, duty_c_0,
    output wire [9:0] duty_a_1, duty_b_1, duty_c_1,
    output wire [9:0] duty_a_2, duty_b_2, duty_c_2,
    output wire [9:0] duty_a_3, duty_b_3, duty_c_3,
    output wire [9:0] duty_a_4, duty_b_4, duty_c_4,
    output wire [9:0] duty_a_5, duty_b_5, duty_c_5
);

    wire init_done_0, init_done_1, init_done_2;
    wire init_done_3, init_done_4, init_done_5;

    assign all_init_done = init_done_0 & init_done_1 & init_done_2 &
                           init_done_3 & init_done_4 & init_done_5;

    // Unused id feedback wires
    wire signed [15:0] id_actual_0, id_actual_1, id_actual_2;
    wire signed [15:0] id_actual_3, id_actual_4, id_actual_5;
    wire en_idq_0, en_idq_1, en_idq_2, en_idq_3, en_idq_4, en_idq_5;

    // ======================== Channel 0 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_0),
        .POLE_PAIR(POLE_PAIR_0), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch0 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_0), .phi(phi_0),
        .sn_adc(sn_adc_0), .en_adc(en_adc_0),
        .adc_a(adc_a_0), .adc_b(adc_b_0), .adc_c(adc_c_0),
        .pwm_en(pwm_en_0), .pwm_a(pwm_a_0), .pwm_b(pwm_b_0), .pwm_c(pwm_c_0),
        .en_idq(en_idq_0), .id_actual(id_actual_0), .iq_actual(iq_actual_0),
        .init_done(init_done_0),
        .duty_a(duty_a_0), .duty_b(duty_b_0), .duty_c(duty_c_0)
    );

    // ======================== Channel 1 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_1),
        .POLE_PAIR(POLE_PAIR_1), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch1 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_1), .phi(phi_1),
        .sn_adc(sn_adc_1), .en_adc(en_adc_1),
        .adc_a(adc_a_1), .adc_b(adc_b_1), .adc_c(adc_c_1),
        .pwm_en(pwm_en_1), .pwm_a(pwm_a_1), .pwm_b(pwm_b_1), .pwm_c(pwm_c_1),
        .en_idq(en_idq_1), .id_actual(id_actual_1), .iq_actual(iq_actual_1),
        .init_done(init_done_1),
        .duty_a(duty_a_1), .duty_b(duty_b_1), .duty_c(duty_c_1)
    );

    // ======================== Channel 2 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_2),
        .POLE_PAIR(POLE_PAIR_2), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch2 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_2), .phi(phi_2),
        .sn_adc(sn_adc_2), .en_adc(en_adc_2),
        .adc_a(adc_a_2), .adc_b(adc_b_2), .adc_c(adc_c_2),
        .pwm_en(pwm_en_2), .pwm_a(pwm_a_2), .pwm_b(pwm_b_2), .pwm_c(pwm_c_2),
        .en_idq(en_idq_2), .id_actual(id_actual_2), .iq_actual(iq_actual_2),
        .init_done(init_done_2),
        .duty_a(duty_a_2), .duty_b(duty_b_2), .duty_c(duty_c_2)
    );

    // ======================== Channel 3 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_3),
        .POLE_PAIR(POLE_PAIR_3), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch3 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_3), .phi(phi_3),
        .sn_adc(sn_adc_3), .en_adc(en_adc_3),
        .adc_a(adc_a_3), .adc_b(adc_b_3), .adc_c(adc_c_3),
        .pwm_en(pwm_en_3), .pwm_a(pwm_a_3), .pwm_b(pwm_b_3), .pwm_c(pwm_c_3),
        .en_idq(en_idq_3), .id_actual(id_actual_3), .iq_actual(iq_actual_3),
        .init_done(init_done_3),
        .duty_a(duty_a_3), .duty_b(duty_b_3), .duty_c(duty_c_3)
    );

    // ======================== Channel 4 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_4),
        .POLE_PAIR(POLE_PAIR_4), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch4 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_4), .phi(phi_4),
        .sn_adc(sn_adc_4), .en_adc(en_adc_4),
        .adc_a(adc_a_4), .adc_b(adc_b_4), .adc_c(adc_c_4),
        .pwm_en(pwm_en_4), .pwm_a(pwm_a_4), .pwm_b(pwm_b_4), .pwm_c(pwm_c_4),
        .en_idq(en_idq_4), .id_actual(id_actual_4), .iq_actual(iq_actual_4),
        .init_done(init_done_4),
        .duty_a(duty_a_4), .duty_b(duty_b_4), .duty_c(duty_c_4)
    );

    // ======================== Channel 5 ========================
    foc_channel #(
        .INIT_CYCLES(INIT_CYCLES), .ANGLE_INV(ANGLE_INV_5),
        .POLE_PAIR(POLE_PAIR_5), .MAX_AMP(MAX_AMP), .SAMPLE_DELAY(SAMPLE_DELAY)
    ) ch5 (
        .foc_clk(foc_clk), .rst_n(rst_n),
        .Kp(Kp), .Ki(Ki),
        .iq_ref(iq_ref_5), .phi(phi_5),
        .sn_adc(sn_adc_5), .en_adc(en_adc_5),
        .adc_a(adc_a_5), .adc_b(adc_b_5), .adc_c(adc_c_5),
        .pwm_en(pwm_en_5), .pwm_a(pwm_a_5), .pwm_b(pwm_b_5), .pwm_c(pwm_c_5),
        .en_idq(en_idq_5), .id_actual(id_actual_5), .iq_actual(iq_actual_5),
        .init_done(init_done_5),
        .duty_a(duty_a_5), .duty_b(duty_b_5), .duty_c(duty_c_5)
    );

endmodule
