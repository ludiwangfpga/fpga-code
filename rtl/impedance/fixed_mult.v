`timescale 1ns / 1ps

// fixed_mult.v - Q16.16 Fixed Point Multiplier
// Performs signed multiplication with proper scaling
// Output = (A * B) >> 16

module fixed_mult (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] a,      // Q16.16 multiplicand
    input  wire signed [31:0] b,      // Q16.16 multiplier
    output reg  signed [31:0] result, // Q16.16 product
    output reg         done
);

    // Internal 64-bit product
    reg signed [63:0] product;
    reg [1:0] state;

    localparam IDLE = 2'd0;
    localparam CALC = 2'd1;
    localparam DONE = 2'd2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            result <= 32'sd0;
            product <= 64'sd0;
            done <= 1'b0;
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        product <= $signed(a) * $signed(b);
                        state <= CALC;
                    end
                end

                CALC: begin
                    // Arithmetic right shift by 16 to scale result
                    result <= product[47:16];
                    state <= DONE;
                end

                DONE: begin
                    done <= 1'b1;
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule

// Combinational version for simple use cases
module fixed_mult_comb (
    input  wire signed [31:0] a,
    input  wire signed [31:0] b,
    output wire signed [31:0] result
);

    wire signed [63:0] product;

    assign product = $signed(a) * $signed(b);
    assign result = product[47:16];

endmodule
