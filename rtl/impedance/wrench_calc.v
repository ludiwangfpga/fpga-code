`timescale 1ns / 1ps
// wrench_calc.v — F = K·Δx − D·ẋ (M4.3 刚度+阻尼). Δx[i]=x_des[i]−pose[i]. 对角 K/D.
//   流水: S_IDLE(锁存 Δx) -> S_MUL(kp=fmul64(K,Δx), dv=fmul64(D,ẋ) 各 64位) -> S_SUB(F=sat32(kp-dv)) -> S_DONE.
//   注: Δx=x_des−pose 用 32 位差(假设 |x_des−pose| 不溢出 int32, 即位姿/期望在合理量程, m/rad 远小于 ±32768); 之后乘加均 64 位中间值 + sat32 输出饱和.
module wrench_calc (
    input  wire clk, rst_n, start,
    input  wire signed [31:0] xd0,xd1,xd2,xd3,xd4,xd5,   // x_des
    input  wire signed [31:0] p0,p1,p2,p3,p4,p5,          // pose
    input  wire signed [31:0] K0,K1,K2,K3,K4,K5,
    input  wire signed [31:0] D0,D1,D2,D3,D4,D5,
    input  wire signed [31:0] xv0,xv1,xv2,xv3,xv4,xv5,    // ẋ (笛卡尔速度)
    output reg  signed [31:0] F0,F1,F2,F3,F4,F5,
    output reg  done
);
    localparam S_IDLE=2'd0, S_MUL=2'd1, S_SUB=2'd2, S_DONE=2'd3;
    reg [1:0] state;
    reg signed [31:0] dx0,dx1,dx2,dx3,dx4,dx5;
    reg signed [63:0] kp0,kp1,kp2,kp3,kp4,kp5;   // K·Δx (64位)
    reg signed [63:0] dv0,dv1,dv2,dv3,dv4,dv5;   // D·ẋ  (64位)
    (* use_dsp = "yes" *)
    function signed [63:0] fmul64; input signed [31:0] a,b; reg signed [63:0] t;
        begin t=$signed(a)*$signed(b); fmul64 = t>>>16; end endfunction
    function signed [31:0] sat32; input signed [63:0] v;
        begin if (v>64'sd2147483647) sat32=32'sd2147483647;
              else if (v<-64'sd2147483647) sat32=-32'sd2147483647;
              else sat32=v[31:0]; end endfunction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin state<=S_IDLE; done<=0; F0<=0;F1<=0;F2<=0;F3<=0;F4<=0;F5<=0; end
        else case(state)
            S_IDLE: begin done<=0; if (start) begin
                dx0<=xd0-p0; dx1<=xd1-p1; dx2<=xd2-p2; dx3<=xd3-p3; dx4<=xd4-p4; dx5<=xd5-p5;
                state<=S_MUL; end end
            S_MUL: begin
                kp0<=fmul64(K0,dx0); kp1<=fmul64(K1,dx1); kp2<=fmul64(K2,dx2);
                kp3<=fmul64(K3,dx3); kp4<=fmul64(K4,dx4); kp5<=fmul64(K5,dx5);
                dv0<=fmul64(D0,xv0); dv1<=fmul64(D1,xv1); dv2<=fmul64(D2,xv2);
                dv3<=fmul64(D3,xv3); dv4<=fmul64(D4,xv4); dv5<=fmul64(D5,xv5);
                state<=S_SUB; end
            S_SUB: begin
                F0<=sat32(kp0-dv0); F1<=sat32(kp1-dv1); F2<=sat32(kp2-dv2);
                F3<=sat32(kp3-dv3); F4<=sat32(kp4-dv4); F5<=sat32(kp5-dv5);
                state<=S_DONE; end
            S_DONE: begin done<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
        endcase
    end
endmodule
