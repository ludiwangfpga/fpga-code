`timescale 1ns / 1ps
// foc_motor_model.v - 单电机 + ADC 行为模型(仅仿真).
//   读三相占空比(duty_a/b/c, 中心512) -> 相电压 va/vb/vc -> 一阶低通(电感+电阻)电流 ia/ib/ic
//   -> adc_x = 2048 - (i_x>>>IS), 与 foc_top 的 Ia=ADCb+ADCc-2*ADCa 配套(得 3k*i_a, 自洽负反馈).
//   sn_adc 脉冲 -> 延 DLY 拍 -> en_adc 一拍高 + adc_a/b/c 输出.
//
//   一阶电流模型: i[n+1] = i[n] + ((v*KV - i[n]) >>> SH)
//     KV 越大稳态电流越大(等效 1/R), SH 越大电流越慢(等效 L/R 时间常数).
//   ADC 偏置方向(2048 -/+ ...)决定电流环正/负反馈, 见 ADC_SIGN 参数.
module foc_motor_model #(
    parameter integer KV       = 4,   // 电压->稳态电流增益(等效电导)
    parameter integer SH       = 6,   // 一阶滞后移位(时间常数, 越大越慢)
    parameter integer IS       = 2,   // 电流->ADC 缩放右移
    parameter integer DLY      = 20,  // sn_adc 到 en_adc 的延时拍数
    parameter integer ADC_SIGN = -1   // -1: adc=2048-(i>>>IS) (负反馈); +1: adc=2048+(i>>>IS)
)(
    input  wire        clk, rst_n,
    input  wire [9:0]  duty_a, duty_b, duty_c,
    input  wire        sn_adc,
    output reg         en_adc,
    output reg  [11:0] adc_a, adc_b, adc_c
);
    reg signed [31:0] ia, ib, ic;
    reg        [7:0]  dly_cnt;
    reg               sampling;

    // 相电压 = 占空比 - 中心512 (有符号)
    wire signed [31:0] va = $signed({22'd0, duty_a}) - 32'sd512;
    wire signed [31:0] vb = $signed({22'd0, duty_b}) - 32'sd512;
    wire signed [31:0] vc = $signed({22'd0, duty_c}) - 32'sd512;

    function [11:0] clamp12;
        input signed [31:0] v;
        begin
            if(v < 0)                  clamp12 = 12'd0;
            else if(v > 32'sd4095)     clamp12 = 12'd4095;
            else                       clamp12 = v[11:0];
        end
    endfunction

    // 由电流算 ADC 值(含偏置方向)
    function [11:0] cur2adc;
        input signed [31:0] i;
        reg   signed [31:0] off;
        begin
            off = (i >>> IS);
            if(ADC_SIGN < 0) cur2adc = clamp12(32'sd2048 - off);
            else             cur2adc = clamp12(32'sd2048 + off);
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            ia <= 0; ib <= 0; ic <= 0;
            en_adc <= 1'b0;
            adc_a <= 12'd2048; adc_b <= 12'd2048; adc_c <= 12'd2048;
            dly_cnt <= 8'd0; sampling <= 1'b0;
        end else begin
            // 一阶电流积分(连续运行, 不受采样门控)
            ia <= ia + (((va * KV) - ia) >>> SH);
            ib <= ib + (((vb * KV) - ib) >>> SH);
            ic <= ic + (((vc * KV) - ic) >>> SH);

            en_adc <= 1'b0;
            if(sn_adc) begin
                sampling <= 1'b1;
                dly_cnt  <= DLY[7:0];
            end else if(sampling) begin
                if(dly_cnt == 8'd0) begin
                    adc_a <= cur2adc(ia);
                    adc_b <= cur2adc(ib);
                    adc_c <= cur2adc(ic);
                    en_adc   <= 1'b1;
                    sampling <= 1'b0;
                end else begin
                    dly_cnt <= dly_cnt - 8'd1;
                end
            end
        end
    end
endmodule
