`timescale 1ns / 1ps

// cordic_sincos.v - CORDIC Sin/Cos Calculator
// Computes sin(angle) and cos(angle) using CORDIC algorithm
// Input angle in Q16.16 radians, output in Q16.16

module cordic_sincos (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] angle,    // Input angle in radians (Q16.16)
    output reg  signed [31:0] sin_out,  // sin(angle) in Q16.16
    output reg  signed [31:0] cos_out,  // cos(angle) in Q16.16
    output reg         done
);

    // CORDIC iterations
    parameter ITERATIONS = 16;

    // State machine
    localparam IDLE      = 3'd0;
    localparam INIT      = 3'd1;
    localparam ROTATE    = 3'd2;
    localparam SCALE     = 3'd3;
    localparam DONE_ST   = 3'd4;

    reg [2:0] state;
    reg [4:0] iter;

    // CORDIC working registers
    reg signed [31:0] x, y, z;
    reg signed [31:0] x_new, y_new;

    // Sign flags for final adjustment
    reg negate_sin;
    reg negate_cos;

    // CORDIC arctangent lookup table (atan(2^-i) in Q16.16)
    wire signed [31:0] atan_table [0:15];
    assign atan_table[0]  = 32'sd51472;   // atan(1)     = 45.0°
    assign atan_table[1]  = 32'sd30386;   // atan(0.5)   = 26.57°
    assign atan_table[2]  = 32'sd16055;   // atan(0.25)  = 14.04°
    assign atan_table[3]  = 32'sd8150;    // atan(0.125) = 7.13°
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
    localparam signed [31:0] PI     = 32'sd205887;    // π
    localparam signed [31:0] PI_2   = 32'sd102944;    // π/2
    localparam signed [31:0] NEG_PI_2 = -32'sd102944; // -π/2

    // CORDIC gain inverse (1/K ≈ 0.6073)
    localparam signed [31:0] CORDIC_K = 32'sd39797;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sin_out <= 32'sd0;
            cos_out <= 32'sd0;
            done <= 1'b0;
            state <= IDLE;
            iter <= 5'd0;
            x <= 32'sd0;
            y <= 32'sd0;
            z <= 32'sd0;
            negate_sin <= 1'b0;
            negate_cos <= 1'b0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 1'b0;
                    if (start) begin
                        state <= INIT;
                    end
                end

                INIT: begin
                    // Reduce angle to [-π/2, π/2] range for CORDIC
                    // Initialize CORDIC: x = K, y = 0
                    x <= CORDIC_K;
                    y <= 32'sd0;
                    iter <= 5'd0;

                    // Default: no negation
                    negate_sin <= 1'b0;
                    negate_cos <= 1'b0;

                    if (angle >= NEG_PI_2 && angle <= PI_2) begin
                        // Already in range [-π/2, π/2]
                        z <= angle;
                    end else if (angle > PI_2 && angle <= PI) begin
                        // (π/2, π]: sin(θ) = sin(π-θ), cos(θ) = -cos(π-θ)
                        z <= PI - angle;
                        negate_cos <= 1'b1;
                    end else if (angle > PI) begin
                        // (π, 2π): sin(θ) = -sin(θ-π), cos(θ) = -cos(θ-π)
                        z <= angle - PI;
                        negate_sin <= 1'b1;
                        negate_cos <= 1'b1;
                    end else if (angle < NEG_PI_2 && angle >= -PI) begin
                        // [-π, -π/2): sin(θ) = sin(-π-θ), cos(θ) = -cos(-π-θ)
                        // Actually: sin(θ) = -sin(π+θ), cos(θ) = -cos(π+θ)
                        z <= PI + angle;  // This gives a value in [-π/2, 0]
                        negate_sin <= 1'b1;
                        negate_cos <= 1'b1;
                    end else begin
                        // < -π: normalize
                        z <= angle + (PI <<< 1);
                    end

                    state <= ROTATE;
                end

                ROTATE: begin
                    // CORDIC rotation iteration
                    if (z >= 0) begin
                        // Rotate counter-clockwise
                        x_new = x - (y >>> iter);
                        y_new = y + (x >>> iter);
                        z <= z - atan_table[iter];
                    end else begin
                        // Rotate clockwise
                        x_new = x + (y >>> iter);
                        y_new = y - (x >>> iter);
                        z <= z + atan_table[iter];
                    end

                    x <= x_new;
                    y <= y_new;

                    if (iter == ITERATIONS - 1) begin
                        state <= SCALE;
                    end else begin
                        iter <= iter + 1;
                    end
                end

                SCALE: begin
                    // After CORDIC: x ≈ cos(reduced_angle), y ≈ sin(reduced_angle)
                    // Apply sign corrections based on original quadrant
                    sin_out <= negate_sin ? -y : y;
                    cos_out <= negate_cos ? -x : x;
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
