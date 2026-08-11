`timescale 1ns / 1ps

// torque_to_iq.v - Joint Torque to Iq Reference Converter
//
// Converts 6-DOF joint torques (Q16.16) to signed 16-bit Iq current references
// for FOC motor controllers.
//
//   iq_ref[i] = clamp(tau[i] * KT_INV[i], IQ_MIN, IQ_MAX)
//
// where KT_INV = 1/Kt is the inverse of the motor torque constant.
//
// Q16.16 torque * Q16.16 KT_INV → Q32.32 → truncate to signed 16-bit
// Latency: 1 clock cycle (registered output with saturation)

module torque_to_iq #(
    // Inverse torque constants (1/Kt) for each joint, in Q16.16
    // Default: Kt=0.05 Nm/A → KT_INV=20.0 → 20*65536 = 1310720
    parameter signed [31:0] KT_INV_0 = 32'sd1310720,
    parameter signed [31:0] KT_INV_1 = 32'sd1310720,
    parameter signed [31:0] KT_INV_2 = 32'sd1310720,
    parameter signed [31:0] KT_INV_3 = 32'sd1310720,
    parameter signed [31:0] KT_INV_4 = 32'sd1310720,
    parameter signed [31:0] KT_INV_5 = 32'sd1310720,

    // Current limits (signed 16-bit)
    parameter signed [15:0] IQ_MAX =  16'sd2000,
    parameter signed [15:0] IQ_MIN = -16'sd2000
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,

    // Joint torques (Q16.16, Nm)
    input  wire signed [31:0] tau0, tau1, tau2,
    input  wire signed [31:0] tau3, tau4, tau5,

    // Iq current references (signed 16-bit, for FOC iq_aim)
    output reg  signed [15:0] iq0, iq1, iq2,
    output reg  signed [15:0] iq3, iq4, iq5,

    output reg         done
);

    // Q16.16 multiply: result = (a * b) >> 16
    (* use_dsp = "yes" *)
    function signed [31:0] fmult;
        input signed [31:0] a;
        input signed [31:0] b;
        reg signed [63:0] temp;
        begin
            temp = $signed(a) * $signed(b);
            fmult = temp[47:16];
        end
    endfunction

    // Saturate Q16.16 value to signed 16-bit range
    // 必须用 $signed() 保证有符号比较; 直接拼接 {{...},IQ_MAX} 是无符号值, 会导致负数误判为溢出
    function signed [15:0] saturate;
        input signed [31:0] val;
        begin
            if (val > $signed({{16{IQ_MAX[15]}}, IQ_MAX}))
                saturate = IQ_MAX;
            else if (val < $signed({{16{IQ_MIN[15]}}, IQ_MIN}))
                saturate = IQ_MIN;
            else
                saturate = val[15:0];
        end
    endfunction

    // Intermediate Q16.16 results
    reg signed [31:0] iq_q16_0, iq_q16_1, iq_q16_2;
    reg signed [31:0] iq_q16_3, iq_q16_4, iq_q16_5;

    // State machine
    localparam IDLE = 2'd0;
    localparam CALC = 2'd1;
    localparam DONE_ST = 2'd2;

    reg [1:0] state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            done <= 1'b0;
            iq0 <= 0; iq1 <= 0; iq2 <= 0;
            iq3 <= 0; iq4 <= 0; iq5 <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        // Multiply torque by inverse torque constant
                        iq_q16_0 <= fmult(tau0, KT_INV_0);
                        iq_q16_1 <= fmult(tau1, KT_INV_1);
                        iq_q16_2 <= fmult(tau2, KT_INV_2);
                        iq_q16_3 <= fmult(tau3, KT_INV_3);
                        iq_q16_4 <= fmult(tau4, KT_INV_4);
                        iq_q16_5 <= fmult(tau5, KT_INV_5);
                        state <= CALC;
                    end
                end

                CALC: begin
                    // Saturate to 16-bit signed range
                    iq0 <= saturate(iq_q16_0);
                    iq1 <= saturate(iq_q16_1);
                    iq2 <= saturate(iq_q16_2);
                    iq3 <= saturate(iq_q16_3);
                    iq4 <= saturate(iq_q16_4);
                    iq5 <= saturate(iq_q16_5);
                    state <= DONE_ST;
                end

                DONE_ST: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
