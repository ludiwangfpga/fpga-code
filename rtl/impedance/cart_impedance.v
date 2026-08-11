`timescale 1ns / 1ps
// cart_impedance.v — start/done 包装: wrench_calc(F=K·Δx−D·ẋ) -> impedance_torque(τ=Jᵀ·F)
//   仿 analytical_jacobian_serial 包装 fk_engine. M4.3 全量(刚度+阻尼)版.
module cart_impedance (
    input  wire clk, rst_n, start,
    input  wire signed [31:0] J00,J01,J02,J03,J04,J05,
    input  wire signed [31:0] J10,J11,J12,J13,J14,J15,
    input  wire signed [31:0] J20,J21,J22,J23,J24,J25,
    input  wire signed [31:0] J30,J31,J32,J33,J34,J35,
    input  wire signed [31:0] J40,J41,J42,J43,J44,J45,
    input  wire signed [31:0] J50,J51,J52,J53,J54,J55,
    input  wire signed [31:0] p0,p1,p2,p3,p4,p5,
    input  wire signed [31:0] xd0,xd1,xd2,xd3,xd4,xd5,
    input  wire signed [31:0] K0,K1,K2,K3,K4,K5,
    input  wire signed [31:0] D0,D1,D2,D3,D4,D5,
    input  wire signed [31:0] xv0,xv1,xv2,xv3,xv4,xv5,
    output reg  signed [31:0] tau0,tau1,tau2,tau3,tau4,tau5,
    output reg  done
);
    localparam S_IDLE=2'd0, S_WC=2'd1, S_IT=2'd2, S_DONE=2'd3;
    reg [1:0] state; reg wc_start, it_start;
    wire wc_done, it_done;
    wire signed [31:0] Fw0,Fw1,Fw2,Fw3,Fw4,Fw5;
    wire signed [31:0] t0,t1,t2,t3,t4,t5;

    wrench_calc wc(.clk(clk),.rst_n(rst_n),.start(wc_start),
        .xd0(xd0),.xd1(xd1),.xd2(xd2),.xd3(xd3),.xd4(xd4),.xd5(xd5),
        .p0(p0),.p1(p1),.p2(p2),.p3(p3),.p4(p4),.p5(p5),
        .K0(K0),.K1(K1),.K2(K2),.K3(K3),.K4(K4),.K5(K5),
        .D0(D0),.D1(D1),.D2(D2),.D3(D3),.D4(D4),.D5(D5),
        .xv0(xv0),.xv1(xv1),.xv2(xv2),.xv3(xv3),.xv4(xv4),.xv5(xv5),
        .F0(Fw0),.F1(Fw1),.F2(Fw2),.F3(Fw3),.F4(Fw4),.F5(Fw5),.done(wc_done));
    impedance_torque it(.clk(clk),.rst_n(rst_n),.start(it_start),
        .J00(J00),.J01(J01),.J02(J02),.J03(J03),.J04(J04),.J05(J05),
        .J10(J10),.J11(J11),.J12(J12),.J13(J13),.J14(J14),.J15(J15),
        .J20(J20),.J21(J21),.J22(J22),.J23(J23),.J24(J24),.J25(J25),
        .J30(J30),.J31(J31),.J32(J32),.J33(J33),.J34(J34),.J35(J35),
        .J40(J40),.J41(J41),.J42(J42),.J43(J43),.J44(J44),.J45(J45),
        .J50(J50),.J51(J51),.J52(J52),.J53(J53),.J54(J54),.J55(J55),
        .F0(Fw0),.F1(Fw1),.F2(Fw2),.F3(Fw3),.F4(Fw4),.F5(Fw5),
        .tau0(t0),.tau1(t1),.tau2(t2),.tau3(t3),.tau4(t4),.tau5(t5),.done(it_done));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_IDLE; done<=0; wc_start<=0; it_start<=0;
            tau0<=0;tau1<=0;tau2<=0;tau3<=0;tau4<=0;tau5<=0;
        end else begin
            wc_start<=0; it_start<=0;
            case(state)
            S_IDLE: begin done<=0; if (start) begin wc_start<=1; state<=S_WC; end end
            S_WC:   if (wc_done) begin it_start<=1; state<=S_IT; end
            S_IT:   if (it_done) begin
                        tau0<=t0;tau1<=t1;tau2<=t2;tau3<=t3;tau4<=t4;tau5<=t5; state<=S_DONE;
                    end
            S_DONE: begin done<=1; state<=S_IDLE; end
            default: state<=S_IDLE;
            endcase
        end
    end
endmodule
