`timescale 1ns / 1ps

// format_converter.v - 定点数格式转换模块
// 实现 S3.12, S4.19, S8.15 与 Q16.16 之间的双向转换
// 所有转换为纯组合逻辑，零延迟

// ============================================================
// S3.12 (16-bit) <-> Q16.16 (32-bit)
// S3.12: 1符号 + 3整数 + 12小数 = 16位, 缩放因子 4096
// Q16.16: 1符号 + 15整数 + 16小数 = 32位, 缩放因子 65536
// 转换: Q16.16 = S3.12 << 4 (左移4位, 即乘以16)
//        S3.12 = Q16.16 >> 4 (右移4位, 截断低4位)
// ============================================================

module s3_12_to_q16_16 (
    input  wire signed [15:0] in_s3_12,
    output wire signed [31:0] out_q16_16
);
    // 符号扩展16位到32位，然后左移4位
    assign out_q16_16 = {{12{in_s3_12[15]}}, in_s3_12, 4'b0000};
endmodule

module q16_16_to_s3_12 (
    input  wire signed [31:0] in_q16_16,
    output wire signed [15:0] out_s3_12,
    output wire               overflow   // 溢出标志
);
    // 右移4位（算术右移）
    wire signed [31:0] shifted = in_q16_16 >>> 4;

    // 检查是否溢出 S3.12 的范围 [-8, 7.999...]
    // S3.12 最大值 = 0x7FFF = 32767 -> 7.9998
    // S3.12 最小值 = 0x8000 = -32768 -> -8.0
    // 溢出条件: shifted 的高16位不全为符号扩展
    assign overflow = (shifted[31:16] != {16{shifted[15]}});

    // 饱和处理
    assign out_s3_12 = overflow ?
        (shifted[31] ? 16'sh8000 : 16'sh7FFF) :
        shifted[15:0];
endmodule


// ============================================================
// S4.19 (24-bit) <-> Q16.16 (32-bit)
// S4.19: 1符号 + 4整数 + 19小数 = 24位, 缩放因子 524288
// Q16.16: 1符号 + 15整数 + 16小数 = 32位, 缩放因子 65536
// 转换: Q16.16 = S4.19 >> 3 (右移3位, 19-16=3)
//        S4.19 = Q16.16 << 3 (左移3位)
// ============================================================

module s4_19_to_q16_16 (
    input  wire signed [23:0] in_s4_19,
    output wire signed [31:0] out_q16_16
);
    // 符号扩展24位到32位，然后算术右移3位
    wire signed [31:0] extended = {{8{in_s4_19[23]}}, in_s4_19};
    assign out_q16_16 = extended >>> 3;
endmodule

module q16_16_to_s4_19 (
    input  wire signed [31:0] in_q16_16,
    output wire signed [23:0] out_s4_19,
    output wire               overflow
);
    // 左移3位
    wire signed [31:0] shifted = in_q16_16 <<< 3;

    // 检查是否溢出 S4.19 的范围 [-16, 15.999...]
    // 溢出条件: shifted 的高8位不全为符号扩展
    assign overflow = (shifted[31:24] != {8{shifted[23]}});

    // 饱和处理
    assign out_s4_19 = overflow ?
        (shifted[31] ? 24'sh800000 : 24'sh7FFFFF) :
        shifted[23:0];
endmodule


// ============================================================
// S8.15 (24-bit) <-> Q16.16 (32-bit)
// S8.15: 1符号 + 8整数 + 15小数 = 24位, 缩放因子 32768
// Q16.16: 1符号 + 15整数 + 16小数 = 32位, 缩放因子 65536
// 转换: Q16.16 = S8.15 << 1 (左移1位, 16-15=1)
//        S8.15 = Q16.16 >> 1 (右移1位)
// ============================================================

module s8_15_to_q16_16 (
    input  wire signed [23:0] in_s8_15,
    output wire signed [31:0] out_q16_16
);
    // 符号扩展24位到32位，然后左移1位
    assign out_q16_16 = {{7{in_s8_15[23]}}, in_s8_15, 1'b0};
endmodule

module q16_16_to_s8_15 (
    input  wire signed [31:0] in_q16_16,
    output wire signed [23:0] out_s8_15,
    output wire               overflow
);
    // 算术右移1位
    wire signed [31:0] shifted = in_q16_16 >>> 1;

    // 检查是否溢出 S8.15 的范围 [-256, 255.999...]
    // 溢出条件: shifted 的高8位不全为符号扩展
    assign overflow = (shifted[31:24] != {8{shifted[23]}});

    // 饱和处理
    assign out_s8_15 = overflow ?
        (shifted[31] ? 24'sh800000 : 24'sh7FFFFF) :
        shifted[23:0];
endmodule


// ============================================================
// S3.20 (24-bit) <-> Q16.16 (32-bit)
// S3.20: 1符号 + 3整数 + 20小数 = 24位, 缩放因子 1048576
// Q16.16: 1符号 + 15整数 + 16小数 = 32位, 缩放因子 65536
// 转换: Q16.16 = S3.20 >> 4 (右移4位, 20-16=4)
//        S3.20 = Q16.16 << 4 (左移4位)
// ============================================================

module s3_20_to_q16_16 (
    input  wire signed [23:0] in_s3_20,
    output wire signed [31:0] out_q16_16
);
    wire signed [31:0] extended = {{8{in_s3_20[23]}}, in_s3_20};
    assign out_q16_16 = extended >>> 4;
endmodule

module q16_16_to_s3_20 (
    input  wire signed [31:0] in_q16_16,
    output wire signed [23:0] out_s3_20,
    output wire               overflow
);
    wire signed [31:0] shifted = in_q16_16 <<< 4;
    assign overflow = (shifted[31:24] != {8{shifted[23]}});
    assign out_s3_20 = overflow ?
        (shifted[31] ? 24'sh800000 : 24'sh7FFFFF) :
        shifted[23:0];
endmodule
