`timescale 1ns / 1ps

// part1_config.v - DH Parameters and Joint Limits Configuration
// Contains ROM for robot configuration constants

module part1_config (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [2:0]  joint_idx,        // Joint index 0-5
    output reg  signed [31:0] dh_d,      // DH parameter d (Q16.16 meters)
    output reg  signed [31:0] dh_a,      // DH parameter a (Q16.16 meters)
    output reg  signed [31:0] dh_alpha,  // DH parameter alpha (Q16.16 radians)
    output reg  signed [31:0] q_min,     // Joint min limit (Q16.16 radians)
    output reg  signed [31:0] q_max,     // Joint max limit (Q16.16 radians)
    output reg  signed [31:0] q_mid      // Joint middle position (Q16.16 radians)
);

    // DH Parameters ROM (Q16.16 format)
    // Joint | d (m)   | a (m)   | alpha (rad)
    // ------+---------+---------+------------
    // 1     | 0.166   | 0.055   | π/2
    // 2     | 0       | 0.200   | 0
    // 3     | 0       | 0.056   | π/2
    // 4     | 0.192   | 0       | -π/2
    // 5     | 0       | 0       | π/2
    // 6     | 0.055   | 0       | 0

    // DH d parameter ROM
    reg signed [31:0] dh_d_rom [0:5];
    // DH a parameter ROM
    reg signed [31:0] dh_a_rom [0:5];
    // DH alpha parameter ROM
    reg signed [31:0] dh_alpha_rom [0:5];
    // Joint limits ROM
    reg signed [31:0] q_min_rom [0:5];
    reg signed [31:0] q_max_rom [0:5];

    // Constants
    localparam signed [31:0] PI_2     = 32'sd102944;   // π/2
    localparam signed [31:0] NEG_PI_2 = -32'sd102944;  // -π/2
    localparam signed [31:0] PI       = 32'sd205887;   // π
    localparam signed [31:0] NEG_PI   = -32'sd205887;  // -π

    // Initialize ROM values
    initial begin
        // DH d parameters (meters in Q16.16)
        dh_d_rom[0] = 32'sd10879;   // 0.166m
        dh_d_rom[1] = 32'sd0;       // 0
        dh_d_rom[2] = 32'sd0;       // 0
        dh_d_rom[3] = 32'sd12583;   // 0.192m
        dh_d_rom[4] = 32'sd0;       // 0
        dh_d_rom[5] = 32'sd3604;    // 0.055m

        // DH a parameters (meters in Q16.16)
        dh_a_rom[0] = 32'sd3604;    // 0.055m
        dh_a_rom[1] = 32'sd13107;   // 0.200m
        dh_a_rom[2] = 32'sd3670;    // 0.056m
        dh_a_rom[3] = 32'sd0;       // 0
        dh_a_rom[4] = 32'sd0;       // 0
        dh_a_rom[5] = 32'sd0;       // 0

        // DH alpha parameters (radians in Q16.16)
        dh_alpha_rom[0] = PI_2;      // π/2
        dh_alpha_rom[1] = 32'sd0;    // 0
        dh_alpha_rom[2] = PI_2;      // π/2
        dh_alpha_rom[3] = NEG_PI_2;  // -π/2
        dh_alpha_rom[4] = PI_2;      // π/2
        dh_alpha_rom[5] = 32'sd0;    // 0

        // Joint minimum limits (radians in Q16.16)
        // Based on motor angle limits converted to DH angles
        q_min_rom[0] = NEG_PI;       // -180° = -π
        q_min_rom[1] = -32'sd68813;  // -60° (approx -1.05 rad)
        q_min_rom[2] = -32'sd154247; // -135° (approx -2.36 rad)
        q_min_rom[3] = -32'sd171476; // -150° (approx -2.62 rad)
        q_min_rom[4] = -32'sd125664; // -110° (approx -1.92 rad)
        q_min_rom[5] = -32'sd171476; // -150° (approx -2.62 rad)

        // Joint maximum limits (radians in Q16.16)
        q_max_rom[0] = PI;           // 180° = π
        q_max_rom[1] = 32'sd240020;  // 210° (approx 3.67 rad)
        q_max_rom[2] = 32'sd154247;  // 135° (approx 2.36 rad)
        q_max_rom[3] = 32'sd171476;  // 150° (approx 2.62 rad)
        q_max_rom[4] = 32'sd125664;  // 110° (approx 1.92 rad)
        q_max_rom[5] = 32'sd171476;  // 150° (approx 2.62 rad)
    end

    // ROM read logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dh_d <= 32'sd0;
            dh_a <= 32'sd0;
            dh_alpha <= 32'sd0;
            q_min <= 32'sd0;
            q_max <= 32'sd0;
            q_mid <= 32'sd0;
        end else begin
            dh_d <= dh_d_rom[joint_idx];
            dh_a <= dh_a_rom[joint_idx];
            dh_alpha <= dh_alpha_rom[joint_idx];
            q_min <= q_min_rom[joint_idx];
            q_max <= q_max_rom[joint_idx];
            // Calculate middle position
            q_mid <= (q_min_rom[joint_idx] + q_max_rom[joint_idx]) >>> 1;
        end
    end

endmodule

// Combinational version for direct access
module part1_config_comb (
    input  wire [2:0]  joint_idx,
    output reg  signed [31:0] dh_d,
    output reg  signed [31:0] dh_a,
    output reg  signed [31:0] dh_alpha,
    output reg  signed [31:0] q_min,
    output reg  signed [31:0] q_max
);

    // Constants
    localparam signed [31:0] PI_2     = 32'sd102944;
    localparam signed [31:0] NEG_PI_2 = -32'sd102944;
    localparam signed [31:0] PI       = 32'sd205887;
    localparam signed [31:0] NEG_PI   = -32'sd205887;

    always @(*) begin
        case (joint_idx)
            3'd0: begin
                dh_d = 32'sd10879;
                dh_a = 32'sd3604;
                dh_alpha = PI_2;
                q_min = NEG_PI;
                q_max = PI;
            end
            3'd1: begin
                dh_d = 32'sd0;
                dh_a = 32'sd13107;
                dh_alpha = 32'sd0;
                q_min = -32'sd68813;
                q_max = 32'sd240020;
            end
            3'd2: begin
                dh_d = 32'sd0;
                dh_a = 32'sd3670;
                dh_alpha = PI_2;
                q_min = -32'sd154247;
                q_max = 32'sd154247;
            end
            3'd3: begin
                dh_d = 32'sd12583;
                dh_a = 32'sd0;
                dh_alpha = NEG_PI_2;
                q_min = -32'sd171476;
                q_max = 32'sd171476;
            end
            3'd4: begin
                dh_d = 32'sd0;
                dh_a = 32'sd0;
                dh_alpha = PI_2;
                q_min = -32'sd125664;
                q_max = 32'sd125664;
            end
            3'd5: begin
                dh_d = 32'sd3604;
                dh_a = 32'sd0;
                dh_alpha = 32'sd0;
                q_min = -32'sd171476;
                q_max = 32'sd171476;
            end
            default: begin
                dh_d = 32'sd0;
                dh_a = 32'sd0;
                dh_alpha = 32'sd0;
                q_min = 32'sd0;
                q_max = 32'sd0;
            end
        endcase
    end

endmodule
