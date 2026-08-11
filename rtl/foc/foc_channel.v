`timescale 1ns / 1ps

// foc_channel.v - Single FOC Motor Channel Wrapper
//
// Wraps foc_top.v with format conversion and parameter adaptation.
// Handles:
//   - Iq reference input (signed 16-bit from torque_to_iq)
//   - Encoder phi input (12-bit from encoder_interface)
//   - ADC interface (directly passed through)
//   - PWM output
//   - Id target fixed at 0 (no field weakening)
//
// Clock: FOC uses its own clock (foc_clk, typically 73.728MHz)
//        Control pipeline uses system clock (sys_clk, typically 100MHz)
//        This module operates in the FOC clock domain.
//        Iq reference crossing is handled by double-flop synchronization.

module foc_channel #(
    parameter        INIT_CYCLES  = 33554432,
    parameter        ANGLE_INV    = 0,
    parameter [ 7:0] POLE_PAIR    = 8'd7,
    parameter [ 8:0] MAX_AMP      = 9'd384,
    parameter [ 8:0] SAMPLE_DELAY = 9'd240
) (
    // FOC clock domain
    input  wire        foc_clk,
    input  wire        rst_n,

    // PI controller parameters
    input  wire [30:0] Kp,
    input  wire [30:0] Ki,

    // Iq reference from control pipeline (signed 16-bit)
    // This signal comes from sys_clk domain; synchronized internally
    input  wire signed [15:0] iq_ref,

    // Encoder input (12-bit mechanical angle, 0-4095)
    input  wire [11:0] phi,

    // ADC interface (directly to/from external ADC)
    output wire        sn_adc,       // ADC sample trigger
    input  wire        en_adc,       // ADC data valid
    input  wire [11:0] adc_a,        // Phase A current ADC
    input  wire [11:0] adc_b,        // Phase B current ADC
    input  wire [11:0] adc_c,        // Phase C current ADC

    // PWM outputs
    output wire        pwm_en,
    output wire        pwm_a,
    output wire        pwm_b,
    output wire        pwm_c,

    // Current feedback (in FOC clock domain)
    output wire               en_idq,
    output wire signed [15:0] id_actual,
    output wire signed [15:0] iq_actual,

    // Status
    output wire        init_done,

    // SVPWM 三相占空比抽头（中心512）
    output wire [9:0]  duty_a, duty_b, duty_c
);

    // Synchronize iq_ref from system clock to FOC clock domain
    // Double-flop synchronizer for the 16-bit value
    // Since iq_ref changes slowly compared to FOC clock, this is safe
    reg signed [15:0] iq_ref_sync1, iq_ref_sync2;

    always @(posedge foc_clk or negedge rst_n) begin
        if (!rst_n) begin
            iq_ref_sync1 <= 16'sd0;
            iq_ref_sync2 <= 16'sd0;
        end else begin
            iq_ref_sync1 <= iq_ref;
            iq_ref_sync2 <= iq_ref_sync1;
        end
    end

    // Id target is always 0 (no field weakening)
    wire signed [15:0] id_aim = 16'sd0;

    // Instantiate FOC core
    foc_top #(
        .INIT_CYCLES  (INIT_CYCLES),
        .ANGLE_INV    (ANGLE_INV),
        .POLE_PAIR    (POLE_PAIR),
        .MAX_AMP      (MAX_AMP),
        .SAMPLE_DELAY (SAMPLE_DELAY)
    ) foc_inst (
        .rstn     (rst_n),
        .clk      (foc_clk),
        .Kp       (Kp),
        .Ki       (Ki),
        .phi      (phi),
        .sn_adc   (sn_adc),
        .en_adc   (en_adc),
        .adc_a    (adc_a),
        .adc_b    (adc_b),
        .adc_c    (adc_c),
        .pwm_en   (pwm_en),
        .pwm_a    (pwm_a),
        .pwm_b    (pwm_b),
        .pwm_c    (pwm_c),
        .en_idq   (en_idq),
        .id       (id_actual),
        .iq       (iq_actual),
        .id_aim   (id_aim),
        .iq_aim   (iq_ref_sync2),
        .init_done(init_done),
        .duty_a   (duty_a),
        .duty_b   (duty_b),
        .duty_c   (duty_c)
    );

endmodule
