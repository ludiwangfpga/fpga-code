`timescale 1ns / 1ps

// cordic_atan2.v - CORDIC Arctangent2 Calculator
// Computes atan2(y, x) using CORDIC algorithm in vectoring mode
// Inputs x, y in Q16.16, output angle in Q16.16 radians

module cordic_atan2 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] x_in,     // X coordinate (Q16.16)
    input  wire signed [31:0] y_in,     // Y coordinate (Q16.16)
    output reg  signed [31:0] angle,    // atan2(y,x) in radians (Q16.16)
    output reg  signed [31:0] magnitude, // sqrt(x^2 + y^2) scaled by K
    output reg         done
);

    // CORDIC iterations
    parameter ITERATIONS = 16;

    // State machine
    localparam IDLE      = 3'd0;
    localparam INIT      = 3'd1;
    localparam VECTOR    = 3'd2;
    localparam ADJUST    = 3'd3;
    localparam DONE_ST   = 3'd4;

    reg [2:0] state;
    reg [4:0] iter;

    // CORDIC working registers
    reg signed [31:0] x, y, z;
    reg signed [31:0] x_new, y_new;

    // Quadrant tracking
    reg x_neg, y_neg;

    // CORDIC arctangent lookup table
    wire signed [31:0] atan_table [0:15];
    assign atan_table[0]  = 32'sd51472;
    assign atan_table[1]  = 32'sd30386;
    assign atan_table[2]  = 32'sd16055;
    assign atan_table[3]  = 32'sd8150;
    assign atan_table[4]  = 32'sd4091;
    assign atan_table[5]  = 32'sd2047;
    assign atan_table[6]  = 32'sd1024;
    assign atan_table[7]  = 32'sd512;
    assign atan_table[8]  = 32'sd256;
    assign atan_table[9]  = 32'sd128;
    assign atan_table[10] = 32'sd64;
    assign atan_table[11] = 32'sd32;
    assign atan_table[12] = 32'sd16;
    assign atan_table[13] = 32'sd8;
    assign atan_table[14] = 32'sd4;
    assign atan_table[15] = 32'sd2;

    // Constants
    localparam signed [31:0] PI   = 32'sd205887;
    localparam signed [31:0] PI_2 = 32'sd102944;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle <= 32'sd0;
            magnitude <= 32'sd0;
            done <= 1'b0;
            state <= IDLE;
            iter <= 5'd0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        state <= INIT;
                    end
                end

                INIT: begin
                    // Handle special cases and move to first quadrant
                    x_neg <= x_in[31];
                    y_neg <= y_in[31];

                    // Take absolute values for CORDIC
                    x <= (x_in[31]) ? -x_in : x_in;
                    y <= (y_in[31]) ? -y_in : y_in;
                    z <= 32'sd0;

                    iter <= 5'd0;
                    state <= VECTOR;
                end

                VECTOR: begin
                    // CORDIC vectoring mode: rotate to make y = 0
                    if (y >= 0) begin
                        // y positive: rotate clockwise
                        x_new = x + (y >>> iter);
                        y_new = y - (x >>> iter);
                        z <= z + atan_table[iter];
                    end else begin
                        // y negative: rotate counter-clockwise
                        x_new = x - (y >>> iter);
                        y_new = y + (x >>> iter);
                        z <= z - atan_table[iter];
                    end

                    x <= x_new;
                    y <= y_new;

                    if (iter == ITERATIONS - 1) begin
                        state <= ADJUST;
                    end else begin
                        iter <= iter + 1;
                    end
                end

                ADJUST: begin
                    // Adjust angle based on original quadrant
                    magnitude <= x;  // x now contains magnitude * K

                    if (!x_neg && !y_neg) begin
                        // Quadrant 1: angle is correct
                        angle <= z;
                    end else if (x_neg && !y_neg) begin
                        // Quadrant 2: angle = π - z
                        angle <= PI - z;
                    end else if (x_neg && y_neg) begin
                        // Quadrant 3: angle = -π + z or -(π - z)
                        angle <= -PI + z;
                    end else begin
                        // Quadrant 4: angle = -z
                        angle <= -z;
                    end

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
