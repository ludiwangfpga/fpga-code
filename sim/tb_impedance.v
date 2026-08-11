`timescale 1ns/1ps
// tb_impedance — 阻抗模块位精确单元 tb. 喂已知整数 J/F, 对定点黄金值精确比对(=0 LSB).
module tb_impedance;
    reg clk, rst_n, start;
    integer tc, pc, fl;
    reg clk_run;
    initial begin clk=0; forever #5 clk=~clk; end

    // nontrivial 的量化 J (行优先, = tb gold_j)
    reg signed [31:0] J [0:35];
    reg signed [31:0] F [0:5];
    wire signed [31:0] tau0,tau1,tau2,tau3,tau4,tau5;
    wire done;

    impedance_torque dut(.clk(clk),.rst_n(rst_n),.start(start),
        .J00(J[0]),.J01(J[1]),.J02(J[2]),.J03(J[3]),.J04(J[4]),.J05(J[5]),
        .J10(J[6]),.J11(J[7]),.J12(J[8]),.J13(J[9]),.J14(J[10]),.J15(J[11]),
        .J20(J[12]),.J21(J[13]),.J22(J[14]),.J23(J[15]),.J24(J[16]),.J25(J[17]),
        .J30(J[18]),.J31(J[19]),.J32(J[20]),.J33(J[21]),.J34(J[22]),.J35(J[23]),
        .J40(J[24]),.J41(J[25]),.J42(J[26]),.J43(J[27]),.J44(J[28]),.J45(J[29]),
        .J50(J[30]),.J51(J[31]),.J52(J[32]),.J53(J[33]),.J54(J[34]),.J55(J[35]),
        .F0(F[0]),.F1(F[1]),.F2(F[2]),.F3(F[3]),.F4(F[4]),.F5(F[5]),
        .tau0(tau0),.tau1(tau1),.tau2(tau2),.tau3(tau3),.tau4(tau4),.tau5(tau5),
        .done(done));

    // M4.2 cart_impedance 例化
    reg signed [31:0] P [0:5], XD [0:5], KK [0:5], DD [0:5], XV [0:5];
    wire signed [31:0] c_tau0,c_tau1,c_tau2,c_tau3,c_tau4,c_tau5; wire c_done; reg c_start;
    cart_impedance cdut(.clk(clk),.rst_n(rst_n),.start(c_start),
        .J00(J[0]),.J01(J[1]),.J02(J[2]),.J03(J[3]),.J04(J[4]),.J05(J[5]),
        .J10(J[6]),.J11(J[7]),.J12(J[8]),.J13(J[9]),.J14(J[10]),.J15(J[11]),
        .J20(J[12]),.J21(J[13]),.J22(J[14]),.J23(J[15]),.J24(J[16]),.J25(J[17]),
        .J30(J[18]),.J31(J[19]),.J32(J[20]),.J33(J[21]),.J34(J[22]),.J35(J[23]),
        .J40(J[24]),.J41(J[25]),.J42(J[26]),.J43(J[27]),.J44(J[28]),.J45(J[29]),
        .J50(J[30]),.J51(J[31]),.J52(J[32]),.J53(J[33]),.J54(J[34]),.J55(J[35]),
        .p0(P[0]),.p1(P[1]),.p2(P[2]),.p3(P[3]),.p4(P[4]),.p5(P[5]),
        .xd0(XD[0]),.xd1(XD[1]),.xd2(XD[2]),.xd3(XD[3]),.xd4(XD[4]),.xd5(XD[5]),
        .K0(KK[0]),.K1(KK[1]),.K2(KK[2]),.K3(KK[3]),.K4(KK[4]),.K5(KK[5]),
        .D0(DD[0]),.D1(DD[1]),.D2(DD[2]),.D3(DD[3]),.D4(DD[4]),.D5(DD[5]),
        .xv0(XV[0]),.xv1(XV[1]),.xv2(XV[2]),.xv3(XV[3]),.xv4(XV[4]),.xv5(XV[5]),
        .tau0(c_tau0),.tau1(c_tau1),.tau2(c_tau2),.tau3(c_tau3),.tau4(c_tau4),.tau5(c_tau5),
        .done(c_done));

    integer g;
    task load_J_nontrivial; begin
        J[0]=-32'sd11578; J[1]=32'sd16922; J[2]=32'sd8895; J[3]=32'sd1802; J[4]=-32'sd808; J[5]=0;
        J[6]=32'sd20053; J[7]=32'sd9770; J[8]=32'sd5136; J[9]=-32'sd3121; J[10]=-32'sd466; J[11]=0;
        J[12]=0; J[13]=32'sd19551; J[14]=32'sd10283; J[15]=0; J[16]=32'sd3481; J[17]=0;
        J[18]=0; J[19]=32'sd32768; J[20]=32'sd32768; J[21]=32'sd14689; J[22]=32'sd32768; J[23]=32'sd54822;
        J[24]=0; J[25]=-32'sd56756; J[26]=-32'sd56756; J[27]=32'sd8481; J[28]=-32'sd56756; J[29]=32'sd31652;
        J[30]=32'sd65536; J[31]=0; J[32]=0; J[33]=-32'sd63303; J[34]=0; J[35]=32'sd16962;
    end endtask

    function automatic within0; input signed [31:0] a,b; reg signed [31:0] d;
        begin d=a-b; if (d<0) d=-d; within0=(d==32'sd0); end endfunction

    function automatic within2; input signed [31:0] a,b; reg signed [31:0] d;
        begin d=a-b; if (d<0) d=-d; within2=(d<=32'sd2); end endfunction

    task run_done; input integer tid; begin
        start<=1; @(posedge clk); start<=0;
        g=0; while (done!==1'b1) begin @(posedge clk); g=g+1;
            if (g>500) begin $display("TIMEOUT t%0d", tid); fl=1; disable run_done; end
        end
    end endtask

    task check_tau; input signed [31:0] e0,e1,e2,e3,e4,e5; input integer tid; reg ok; begin
        tc=tc+1; run_done(tid);
        ok = within0(tau0,e0)&&within0(tau1,e1)&&within0(tau2,e2)
           &&within0(tau3,e3)&&within0(tau4,e4)&&within0(tau5,e5);
        if (ok) begin $display("PASS t%0d: tau ok",tid); pc=pc+1; end
        else $display("FAIL t%0d: got=(%0d,%0d,%0d,%0d,%0d,%0d) exp=(%0d,%0d,%0d,%0d,%0d,%0d)",
            tid,tau0,tau1,tau2,tau3,tau4,tau5,e0,e1,e2,e3,e4,e5);
    end endtask

    task cart_check; input signed [31:0] e0,e1,e2,e3,e4,e5; input integer tid; reg ok; begin
        tc=tc+1;
        c_start<=1; @(posedge clk); c_start<=0;
        g=0; while (c_done!==1'b1) begin @(posedge clk); g=g+1; if (g>500) disable cart_check; end
        ok = within2(c_tau0,e0)&&within2(c_tau1,e1)&&within2(c_tau2,e2)
           &&within2(c_tau3,e3)&&within2(c_tau4,e4)&&within2(c_tau5,e5);
        if (ok) begin $display("PASS t%0d: cart tau ok",tid); pc=pc+1; end
        else $display("FAIL t%0d: cart got=(%0d,%0d,%0d,%0d,%0d,%0d) exp=(%0d,%0d,%0d,%0d,%0d,%0d)",
            tid,c_tau0,c_tau1,c_tau2,c_tau3,c_tau4,c_tau5,e0,e1,e2,e3,e4,e5);
    end endtask

    initial begin
        tc=0; pc=0; fl=0; start=0; c_start=0;
        rst_n=0; repeat(6) @(posedge clk); rst_n=1; repeat(2) @(posedge clk);
        load_J_nontrivial;
        // M4.1: F=[1,0.5,-0.5,0.25,0.1,-0.1]
        F[0]=32'sd65536; F[1]=32'sd32768; F[2]=-32'sd32768; F[3]=32'sd16384; F[4]=32'sd6554; F[5]=-32'sd6554;
        check_tau(-32'sd8106, 32'sd14547, 32'sd8837, 32'sd11091, -32'sd266, 32'sd15173, 1);
        // 饱和: F=30000(all) -> tau0,tau5 钳到 +2147483647
        F[0]=32'sd1966080000; F[1]=32'sd1966080000; F[2]=32'sd1966080000;
        F[3]=32'sd1966080000; F[4]=32'sd1966080000; F[5]=32'sd1966080000;
        check_tau(32'sd2147483647, 32'sd667650000, 32'sd9780000, -32'sd1243560000, -32'sd653430000, 32'sd2147483647, 2);

        // M4.2 cart_impedance: 刚度 F=K*(x_des-pose), τ=Jᵀ·F
        load_J_nontrivial;
        P[0]=32'sd20053; P[1]=32'sd11578; P[2]=-32'sd8661; P[3]=-32'sd1; P[4]=-32'sd85786; P[5]=-32'sd171571;
        XD[0]=32'sd23329; XD[1]=32'sd9612; XD[2]=-32'sd7350; XD[3]=-32'sd1; XD[4]=-32'sd79233; XD[5]=-32'sd174848;
        KK[0]=32'sd52428800; KK[1]=32'sd52428800; KK[2]=32'sd52428800; KK[3]=32'sd2621440; KK[4]=32'sd2621440; KK[5]=32'sd2621440;
        for (g=0;g<6;g=g+1) begin DD[g]=0; XV[g]=0; end
        cart_check(-32'sd1075340, 32'sd528121, 32'sd170013, 32'sd307495, -32'sd192427, 32'sd92669, 3);

        // M4.3 阻尼: + D, ẋ
        DD[0]=32'sd3276800; DD[1]=32'sd3276800; DD[2]=32'sd3276800; DD[3]=32'sd327680; DD[4]=32'sd327680; DD[5]=32'sd327680;
        XV[0]=32'sd1311; XV[1]=32'sd655; XV[2]=-32'sd655; XV[3]=32'sd328; XV[4]=-32'sd1311; XV[5]=32'sd655;
        cart_check(-32'sd1077056, 32'sd509587, 32'sd157190, 32'sd310898, -32'sd196142, 32'sd93616, 4);
        // M4.3 退化 D=0,ẋ=0 应等于 M4.2(用例3)结果
        DD[0]=0; DD[1]=0; DD[2]=0; DD[3]=0; DD[4]=0; DD[5]=0;
        XV[0]=0; XV[1]=0; XV[2]=0; XV[3]=0; XV[4]=0; XV[5]=0;
        cart_check(-32'sd1075340, 32'sd528121, 32'sd170013, 32'sd307495, -32'sd192427, 32'sd92669, 5);

        $display("==== %0d/%0d PASS ====", pc, tc);
        if (pc==tc) $display("ALL TESTS PASSED"); else $display("SOME TESTS FAILED");
        $finish;
    end
endmodule
