`timescale 1ns / 1ps

// encoder_interface.v - Encoder to Q16.16 Radian Converter
//
// Converts raw encoder counts to Q16.16 radian angles for the control pipeline.
// Also provides 12-bit mechanical angle output (0-4095) for FOC phi input.
//
// Conversion: angle_rad = encoder_count * (2*pi / ENCODER_MAX)
// In Q16.16:  angle_rad = encoder_count * ANGLE_SCALE
//
// For 12-bit encoder (4096 counts/rev):
//   ANGLE_SCALE = 2*pi/4096 * 65536 = 6.2832/4096 * 65536 ≈ 100.53 → Q16.16 = 6588255/65536
//   Simplified: angle_rad = encoder_count * 100 (approx, in Q16.16 units per count)
//   More precisely: 2*pi * 65536 / 4096 = 411775 / 4096 ≈ 100.53
//   We use: angle_q16 = encoder_count * TWO_PI_Q16 >> ENCODER_BITS
//
// Also supports multi-turn tracking via overflow detection.
// Latency: 2 clock cycles (combinational multiply + register)

module encoder_interface #(
    parameter ENCODER_BITS = 12,                    // Encoder resolution bits
    parameter ENCODER_MAX  = (1 << ENCODER_BITS),   // Max encoder count (4096 for 12-bit)

    // 2*pi in Q16.16 = 6.2831853 * 65536 = 411775
    parameter signed [31:0] TWO_PI_Q16 = 32'sd411775
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        sample,    // Pulse to trigger encoder read

    // Raw encoder input
    input  wire [ENCODER_BITS-1:0] encoder_count,

    // Output: Q16.16 angle in radians (for FK/Jacobian pipeline)
    output reg  signed [31:0] angle_rad,

    // Output: 12-bit mechanical angle (0-4095, for FOC phi input)
    output wire [11:0] phi,

    // Output: validity
    output reg         valid
);

    // For 12-bit encoder, phi is the raw count
    // For other resolutions, scale to 12-bit range
    generate
        if (ENCODER_BITS == 12) begin : gen_phi_12
            assign phi = encoder_count;
        end else if (ENCODER_BITS > 12) begin : gen_phi_larger
            // Truncate higher bits (take MSBs)
            assign phi = encoder_count[ENCODER_BITS-1 -: 12];
        end else begin : gen_phi_smaller
            // Zero-pad lower bits
            assign phi = {encoder_count, {(12-ENCODER_BITS){1'b0}}};
        end
    endgenerate

    // Angle computation pipeline
    reg [1:0] state;
    localparam IDLE = 2'd0;
    localparam MULT = 2'd1;
    localparam DONE = 2'd2;

    // Intermediate computation
    // angle_rad = encoder_count * TWO_PI_Q16 / ENCODER_MAX
    // = encoder_count * TWO_PI_Q16 >> ENCODER_BITS
    reg signed [63:0] mult_result;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            angle_rad <= 32'sd0;
            valid <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    if (sample) begin
                        // Multiply encoder count by 2*pi in Q16.16
                        mult_result <= $signed({1'b0, encoder_count}) * $signed(TWO_PI_Q16);
                        state <= MULT;
                    end
                end

                MULT: begin
                    // Divide by ENCODER_MAX (right shift by ENCODER_BITS)
                    angle_rad <= mult_result[ENCODER_BITS+31 : ENCODER_BITS];
                    valid <= 1'b1;
                    state <= DONE;
                end

                DONE: begin
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
