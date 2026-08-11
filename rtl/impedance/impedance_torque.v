`timescale 1ns / 1ps
// impedance_torque.v — τ = Jᵀ·F (笛卡尔力旋量 -> 关节力矩)
//   τ[i] = Σ_{r=0..5} J[r][i]·F[r]   (J 行优先输入 J00..J55)
//   串行流水: 外层 i=0..5, 内层 r=0..5; 每项分 SEL(锁存MUX)+ MUL(fmul64)+ ADD(64位累加),
//   SEL 级斩断 jsel/fsel→DSP 组合路径; i 完成时 sat32 -> tau[i]. 延迟 ~109 拍 (<1µs).
module impedance_torque (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire signed [31:0] J00,J01,J02,J03,J04,J05,
    input  wire signed [31:0] J10,J11,J12,J13,J14,J15,
    input  wire signed [31:0] J20,J21,J22,J23,J24,J25,
    input  wire signed [31:0] J30,J31,J32,J33,J34,J35,
    input  wire signed [31:0] J40,J41,J42,J43,J44,J45,
    input  wire signed [31:0] J50,J51,J52,J53,J54,J55,
    input  wire signed [31:0] F0,F1,F2,F3,F4,F5,
    output reg  signed [31:0] tau0,tau1,tau2,tau3,tau4,tau5,
    output reg         done
);
    localparam S_IDLE=3'd0, S_SEL=3'd1, S_MUL=3'd2, S_ADD=3'd3, S_DONE=3'd4;
    reg [2:0] state;
    reg [2:0] oi, ri;            // 输出索引 i / 项索引 r (各 0..5)
    reg signed [63:0] acc, pp;   // 64 位累加 / 当前乘积 (a*b)>>>16
    reg signed [31:0] ja, fa;    // 流水线第1级: 锁存 jsel/fsel 输出，斩断 MUX→DSP 组合路径

    (* use_dsp = "yes" *)
    function signed [63:0] fmul64;
        input signed [31:0] a, b; reg signed [63:0] t;
        begin t = $signed(a)*$signed(b); fmul64 = t >>> 16; end
    endfunction
    function signed [31:0] sat32;
        input signed [63:0] v;
        begin
            if      (v >  64'sd2147483647) sat32 =  32'sd2147483647;
            else if (v < -64'sd2147483647) sat32 = -32'sd2147483647;
            else                            sat32 = v[31:0];
        end
    endfunction
    function signed [31:0] jsel;   // J[r][i], J 行优先
        input [2:0] r, i; reg signed [31:0] v;
        begin
            case (r)
            3'd0: case(i) 3'd0:v=J00;3'd1:v=J01;3'd2:v=J02;3'd3:v=J03;3'd4:v=J04;default:v=J05; endcase
            3'd1: case(i) 3'd0:v=J10;3'd1:v=J11;3'd2:v=J12;3'd3:v=J13;3'd4:v=J14;default:v=J15; endcase
            3'd2: case(i) 3'd0:v=J20;3'd1:v=J21;3'd2:v=J22;3'd3:v=J23;3'd4:v=J24;default:v=J25; endcase
            3'd3: case(i) 3'd0:v=J30;3'd1:v=J31;3'd2:v=J32;3'd3:v=J33;3'd4:v=J34;default:v=J35; endcase
            3'd4: case(i) 3'd0:v=J40;3'd1:v=J41;3'd2:v=J42;3'd3:v=J43;3'd4:v=J44;default:v=J45; endcase
            default: case(i) 3'd0:v=J50;3'd1:v=J51;3'd2:v=J52;3'd3:v=J53;3'd4:v=J54;default:v=J55; endcase
            endcase
            jsel = v;
        end
    endfunction
    function signed [31:0] fsel;
        input [2:0] r; reg signed [31:0] v;
        begin case(r) 3'd0:v=F0;3'd1:v=F1;3'd2:v=F2;3'd3:v=F3;3'd4:v=F4;default:v=F5; endcase fsel=v; end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_IDLE; done<=0; oi<=0; ri<=0; acc<=0; pp<=0;
            ja<=0; fa<=0;
            tau0<=0;tau1<=0;tau2<=0;tau3<=0;tau4<=0;tau5<=0;
        end else begin
            case (state)
            S_IDLE: begin done<=0; if (start) begin oi<=0; ri<=0; acc<=0; state<=S_SEL; end end
            S_SEL:  begin ja <= jsel(ri,oi); fa <= fsel(ri); state<=S_MUL; end  // 流水级1: 锁存 MUX 输出
            S_MUL:  begin pp <= fmul64(ja, fa); state<=S_ADD; end               // 流水级2: DSP 读寄存器值
            S_ADD: begin
                if (ri==3'd5) begin       // 第6项: 总和 = acc(前5项) + pp(第6项)
                    case (oi)
                    3'd0: tau0 <= sat32(acc + pp);
                    3'd1: tau1 <= sat32(acc + pp);
                    3'd2: tau2 <= sat32(acc + pp);
                    3'd3: tau3 <= sat32(acc + pp);
                    3'd4: tau4 <= sat32(acc + pp);
                    default: tau5 <= sat32(acc + pp);
                    endcase
                    if (oi==3'd5) state<=S_DONE;
                    else begin oi<=oi+3'd1; ri<=3'd0; acc<=64'sd0; state<=S_SEL; end
                end else begin
                    acc <= acc + pp; ri<=ri+3'd1; state<=S_SEL;
                end
            end
            S_DONE: begin done<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
            endcase
        end
    end
endmodule
