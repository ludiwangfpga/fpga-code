`timescale 1ns / 1ps
// ============================================================================
// tb_r24_chain.v -- R2.4 experiment B (wrapper-level, actually executed)
//   Three instances of r24_chain_wrap driven in lockstep with IDENTICAL random
//   inputs (same control law, same Q16.16 format, same input/output boundary):
//     u_byp0 : SERIAL=0, adaptation never triggered          (baseline)
//     u_byp1 : SERIAL=0, adaptation triggered concurrently   (deployed bypass)
//     u_ser  : SERIAL=1, adaptation on the critical path     (serial reference)
//   Per sample: random joint angles, impedance payload and adaptation inputs
//   (ranges as in tb_r24_adapt_latency.v); adapt_req is random (p=0.5) so the
//   serial design sees beats with and without an update.
//   Cycle semantic: start cycle = 1 ... done observed (inclusive), same as the
//   on-chip op18 counter minus its fixed 4-cycle command latch.
//   Sentinel: R24_B_OK
// ============================================================================
module tb_r24_chain;
    localparam integer N_SMP  = 1000;
    localparam integer CLK_NS = 8;
    localparam [255*8-1:0] CSV = "data/chain_samples.csv";

    reg clk = 0; always #(CLK_NS/2.0) clk = ~clk;
    reg rst_n, start, req, aen;
    reg signed [31:0] q [0:5], xd [0:5], xv [0:5], Kin [0:5], Din [0:5], qe [0:5], te [0:5], ksn [0:5], dsn [0:5];
    wire signed [15:0] iqA [0:5], iqB [0:5], iqC [0:5];
    wire signed [31:0] KsB [0:5], KsC [0:5], KsA [0:5];
    wire doneA, doneB, doneC;

`define PORTS_COMMON \
        .clk(clk), .rst_n(rst_n), .start(start), .adaptive_en(aen), \
        .q0(q[0]),.q1(q[1]),.q2(q[2]),.q3(q[3]),.q4(q[4]),.q5(q[5]), \
        .xd0(xd[0]),.xd1(xd[1]),.xd2(xd[2]),.xd3(xd[3]),.xd4(xd[4]),.xd5(xd[5]), \
        .xv0(xv[0]),.xv1(xv[1]),.xv2(xv[2]),.xv3(xv[3]),.xv4(xv[4]),.xv5(xv[5]), \
        .Kin0(Kin[0]),.Kin1(Kin[1]),.Kin2(Kin[2]),.Kin3(Kin[3]),.Kin4(Kin[4]),.Kin5(Kin[5]), \
        .Din0(Din[0]),.Din1(Din[1]),.Din2(Din[2]),.Din3(Din[3]),.Din4(Din[4]),.Din5(Din[5]), \
        .qe0(qe[0]),.qe1(qe[1]),.qe2(qe[2]),.qe3(qe[3]),.qe4(qe[4]),.qe5(qe[5]), \
        .te0(te[0]),.te1(te[1]),.te2(te[2]),.te3(te[3]),.te4(te[4]),.te5(te[5]), \
        .ksn0(ksn[0]),.ksn1(ksn[1]),.ksn2(ksn[2]),.ksn3(ksn[3]),.ksn4(ksn[4]),.ksn5(ksn[5]), \
        .dsn0(dsn[0]),.dsn1(dsn[1]),.dsn2(dsn[2]),.dsn3(dsn[3]),.dsn4(dsn[4]),.dsn5(dsn[5])

    r24_chain_wrap #(.SERIAL(0)) u_byp0 ( `PORTS_COMMON, .adapt_req(1'b0),
        .iq0(iqA[0]),.iq1(iqA[1]),.iq2(iqA[2]),.iq3(iqA[3]),.iq4(iqA[4]),.iq5(iqA[5]),
        .Ks0(KsA[0]),.Ks1(KsA[1]),.Ks2(KsA[2]),.Ks3(KsA[3]),.Ks4(KsA[4]),.Ks5(KsA[5]), .done(doneA));
    r24_chain_wrap #(.SERIAL(0)) u_byp1 ( `PORTS_COMMON, .adapt_req(req),
        .iq0(iqB[0]),.iq1(iqB[1]),.iq2(iqB[2]),.iq3(iqB[3]),.iq4(iqB[4]),.iq5(iqB[5]),
        .Ks0(KsB[0]),.Ks1(KsB[1]),.Ks2(KsB[2]),.Ks3(KsB[3]),.Ks4(KsB[4]),.Ks5(KsB[5]), .done(doneB));
    r24_chain_wrap #(.SERIAL(1)) u_ser  ( `PORTS_COMMON, .adapt_req(req),
        .iq0(iqC[0]),.iq1(iqC[1]),.iq2(iqC[2]),.iq3(iqC[3]),.iq4(iqC[4]),.iq5(iqC[5]),
        .Ks0(KsC[0]),.Ks1(KsC[1]),.Ks2(KsC[2]),.Ks3(KsC[3]),.Ks4(KsC[4]),.Ks5(KsC[5]), .done(doneC));

    // ---- three start->done counters (same semantic as op18 / tb_r2_1_fwd_cycles) ----
    reg [15:0] cA, cB, cC, lA, lB, lC; reg rA, rB, rC;
    // adaptation busy inside the serial instance (adapt_start -> adapt_done)
    reg [15:0] cAd, lAd; reg rAd;
    always @(posedge clk) begin
        if (!rst_n) begin cA<=0;cB<=0;cC<=0;lA<=0;lB<=0;lC<=0;rA<=0;rB<=0;rC<=0; cAd<=0; lAd<=0; rAd<=0; end
        else begin
            if (start && !rA) begin rA<=1; cA<=1; end else if (rA) begin cA<=cA+1; if (doneA) begin lA<=cA+1; rA<=0; end end
            if (start && !rB) begin rB<=1; cB<=1; end else if (rB) begin cB<=cB+1; if (doneB) begin lB<=cB+1; rB<=0; end end
            if (start && !rC) begin rC<=1; cC<=1; end else if (rC) begin cC<=cC+1; if (doneC) begin lC<=cC+1; rC<=0; end end
            if (u_ser.adapt_start && !rAd) begin rAd<=1; cAd<=1; end
            else if (rAd) begin cAd<=cAd+1; if (u_ser.adapt_done) begin lAd<=cAd+1; rAd<=0; end end
        end
    end

    integer seed;
    function integer urand; input integer lo; input integer hi;
        integer r; begin r = $random(seed); if (r < 0) r = -r; urand = lo + (r % (hi - lo + 1)); end
    endfunction

    integer n, i, fd, g, kenv, eql, sgn, noise, to; reg reqs;
    integer recA [0:N_SMP-1], recB [0:N_SMP-1], recC [0:N_SMP-1], recR [0:N_SMP-1], recAd [0:N_SMP-1];
    integer minA,maxA,minB,maxB,minC,maxC, sumC, medC, acc, iq_eq_BC, iq_ne_AB, nreq, tau_eq_BC, tau_ne_AB;
    integer minC1,maxC1,minC0,maxC0;
    real meanC, varC, sdC, meanB, varB, sdB;
    integer hist [0:4095];

    initial begin
        seed = 777;
        rst_n = 0; start = 0; req = 0; aen = 1;
        for (i=0;i<6;i=i+1) begin q[i]=0; xd[i]=0; xv[i]=0; Kin[i]=32'sd6553600; Din[i]=32'sd655360; qe[i]=0; te[i]=0; ksn[i]=32'sd6553600; dsn[i]=32'sd655360; end
        for (i=0;i<4096;i=i+1) hist[i]=0;
        repeat (5) @(posedge clk); #1 rst_n = 1;
        repeat (5) @(posedge clk);
        fd = $fopen(CSV, "w");
        $fdisplay(fd, "n,adapt_req,adaptive_en,byp_noadapt_cycles,byp_adapt_cycles,serial_cycles,serial_adapt_busy,tau0_noadapt_q16,tau0_byp_q16,tau0_ser_q16,iq0_byp,iq0_ser,Ks0_byp,Ks0_ser");

        for (n=0; n<N_SMP; n=n+1) begin
            req = urand(0,1);
            g   = urand(0,99);
            aen = (g < 10) ? 1'b0 : 1'b1;                       // 10 % beats with adaptive_en=0
            for (i=0;i<6;i=i+1) begin
                q[i]   = urand(-102900, 102900);                // +/- 1.57 rad
                xd[i]  = 0;                                      // set after the warm-up FK pass (pose +/- 0.5 mm)
                xv[i]  = urand(-33, 33);                        // +/- 0.5 mm/s
                Kin[i] = urand(100,5000) * 65536;
                Din[i] = urand(1,100) * 65536;
                kenv   = urand(100,10000);
                eql    = (g >= 10 && g < 20) ? urand(7,64) : urand(66,1966);
                sgn    = urand(0,1) ? 1 : -1; eql = sgn*eql;
                noise  = urand(-20,20);
                qe[i]  = eql;
                te[i]  = (kenv*eql) + ((kenv*eql)/1000)*noise;
                ksn[i] = urand(100,5000) * 65536;
                dsn[i] = urand(1,100) * 65536;
            end
            // warm-up pass (not recorded): run FK once so that x_des can be placed
            // within +/-5 mm / 5 mrad of the actual pose -> non-saturated iq
            reqs = req; req = 1'b0;
            @(posedge clk); #1; start = 1'b1;
            @(posedge clk); #1; start = 1'b0;
            to = 0;
            while (!(doneA===1'b1) && to < 20000) begin @(posedge clk); to=to+1; end
            repeat (3) @(posedge clk); #1;
            for (i=0;i<6;i=i+1) xd[i] = u_byp0.pose_reg[i] + urand(-33, 33);   // pose +/- 0.5 mm / 0.5 mrad
            req = reqs;
            // measured pass
            @(posedge clk); #1; start = 1'b1;
            @(posedge clk); #1; start = 1'b0;
            to = 0;
            while (!(doneA===1'b1) && to < 20000) begin @(posedge clk); to=to+1; end
            to = 0;
            while (!(doneB===1'b1) && to < 20000 && rB) begin @(posedge clk); to=to+1; end
            to = 0;
            while (!(doneC===1'b1) && to < 20000 && rC) begin @(posedge clk); to=to+1; end
            repeat (3) @(posedge clk); #1;
            recA[n]=lA; recB[n]=lB; recC[n]=lC; recR[n]=req; recAd[n]=(req? lAd : 0);
            hist[lC] = hist[lC]+1;
            $fdisplay(fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d", n, req, aen, lA, lB, lC, recAd[n], u_byp0.tau_reg[0], u_byp1.tau_reg[0], u_ser.tau_reg[0], iqB[0], iqC[0], KsB[0], KsC[0]);
            if (u_byp1.tau_reg[0]===u_ser.tau_reg[0] && u_byp1.tau_reg[1]===u_ser.tau_reg[1] && u_byp1.tau_reg[2]===u_ser.tau_reg[2] &&
                u_byp1.tau_reg[3]===u_ser.tau_reg[3] && u_byp1.tau_reg[4]===u_ser.tau_reg[4] && u_byp1.tau_reg[5]===u_ser.tau_reg[5]) tau_eq_BC = tau_eq_BC + 1;
            if (!(u_byp0.tau_reg[0]===u_byp1.tau_reg[0] && u_byp0.tau_reg[1]===u_byp1.tau_reg[1] && u_byp0.tau_reg[2]===u_byp1.tau_reg[2] &&
                  u_byp0.tau_reg[3]===u_byp1.tau_reg[3] && u_byp0.tau_reg[4]===u_byp1.tau_reg[4] && u_byp0.tau_reg[5]===u_byp1.tau_reg[5])) tau_ne_AB = tau_ne_AB + 1;
            if (iqB[0]===iqC[0] && iqB[1]===iqC[1] && iqB[2]===iqC[2] && iqB[3]===iqC[3] && iqB[4]===iqC[4] && iqB[5]===iqC[5]) iq_eq_BC = iq_eq_BC + 1;
            if (!(iqA[0]===iqB[0] && iqA[1]===iqB[1] && iqA[2]===iqB[2] && iqA[3]===iqB[3] && iqA[4]===iqB[4] && iqA[5]===iqB[5])) iq_ne_AB = iq_ne_AB + 1;
            repeat (urand(0,3)) @(posedge clk);
        end
        $fclose(fd);

        minA=100000;maxA=0;minB=100000;maxB=0;minC=100000;maxC=0;sumC=0;nreq=0;
        minC1=100000;maxC1=0;minC0=100000;maxC0=0; meanB=0;
        for (n=0;n<N_SMP;n=n+1) begin
            if (recA[n]<minA) minA=recA[n]; if (recA[n]>maxA) maxA=recA[n];
            if (recB[n]<minB) minB=recB[n]; if (recB[n]>maxB) maxB=recB[n];
            if (recC[n]<minC) minC=recC[n]; if (recC[n]>maxC) maxC=recC[n];
            sumC = sumC + recC[n]; meanB = meanB + recB[n];
            if (recR[n]) begin nreq=nreq+1; if (recC[n]<minC1) minC1=recC[n]; if (recC[n]>maxC1) maxC1=recC[n]; end
            else begin if (recC[n]<minC0) minC0=recC[n]; if (recC[n]>maxC0) maxC0=recC[n]; end
        end
        meanC = sumC*1.0/N_SMP; meanB = meanB/N_SMP; varC = 0; varB = 0;
        for (n=0;n<N_SMP;n=n+1) begin varC = varC + (recC[n]-meanC)*(recC[n]-meanC); varB = varB + (recB[n]-meanB)*(recB[n]-meanB); end
        sdC = $sqrt(varC/N_SMP); sdB = $sqrt(varB/N_SMP);
        acc=0; medC=-1; for (i=0;i<4096;i=i+1) begin acc=acc+hist[i]; if (medC<0 && acc>N_SMP/2) medC=i; end

        $display("=============================================================");
        $display("R2.4-B  wrapper-level serial vs bypass, N=%0d (adapt_req=1 in %0d beats)", N_SMP, nreq);
        $display("  BYPASS, no adaptation      : min=%0d max=%0d", minA, maxA);
        $display("  BYPASS, concurrent adapt   : min=%0d max=%0d mean=%f sd=%f", minB, maxB, meanB, sdB);
        $display("  SERIAL (adapt on path)     : min=%0d median=%0d max=%0d mean=%f sd=%f", minC, medC, maxC, meanC, sdC);
        $display("    serial beats with update : min=%0d max=%0d", minC1, maxC1);
        $display("    serial beats w/o  update : min=%0d max=%0d", minC0, maxC0);
        $display("  serial histogram:");
        for (i=0;i<4096;i=i+1) if (hist[i]>0) $display("    %0d cycles : %0d beats", i, hist[i]);
        $display("  tau(bypass-concurrent) == tau(serial) in %0d / %0d beats (same law, same I/O, 6 channels bit-exact)", tau_eq_BC, N_SMP);
        $display("  iq (bypass-concurrent) == iq (serial) in %0d / %0d beats", iq_eq_BC, N_SMP);
        $display("  tau(no-adapt) != tau(adapt) in %0d / %0d beats (adaptation is not a no-op)", tau_ne_AB, N_SMP);
        $display("  iq (no-adapt) != iq (adapt) in %0d / %0d beats (note: iq clamps at +/-2000 LSB = 0.0305 A, i.e. |tau| > 0.0015 Nm)", iq_ne_AB, N_SMP);
        $display("R24_B_SUMMARY bypA=%0d/%0d bypB=%0d/%0d ser=%0d/%0d/%0d sdser=%f ser1=%0d/%0d ser0=%0d/%0d",
                 minA,maxA,minB,maxB,minC,medC,maxC,sdC,minC1,maxC1,minC0,maxC0);
        if (minA==maxA && minB==maxB && minA==minB && maxC>maxB) $display("R24_B_OK"); else $display("R24_B_FAIL");
        $finish;
    end
    initial begin iq_eq_BC=0; iq_ne_AB=0; tau_eq_BC=0; tau_ne_AB=0; end
    initial begin #400000000; $display("TIMEOUT"); $display("R24_B_FAIL"); $finish; end
endmodule
