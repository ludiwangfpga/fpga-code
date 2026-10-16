`timescale 1ns / 1ps
// ============================================================================
// tb_r24_adapt_latency.v -- R2.4 experiment A
//   Measure the start->done latency distribution of the DEPLOYED adaptation
//   module adaptive_impedance_v2 (rtl/impedance/adaptive_impedance_v2.v of this
//   repository, the file in the deployed bitstream) over N random but
//   physically plausible updates.
//
//   Input ranges (paper Section V): K_env 100..10000 N/m, |e_q| 0.1..30 mm,
//   dead-zone 1 mm (Q_DEADZONE = 65 LSB = 0.00099 in Q16.16), Ks_nom 100..5000,
//   Ds_nom 1..100, tau_ext = K_env*e_q*(1 +/- 2% noise).  All Q16.16.
//
//   Mix: 80% |e_q| in [1,30] mm (active), 10% in [0.1,1) mm (dead-zone),
//        10% adaptive_enable=0 (pass-through).  Every 50 updates a random
//        RLS prior is injected through prior_load (op16 path).
//
//   Cycle semantic = the one used by sim/tb_r2_1_fwd_cycles.v and by the
//   on-chip op18 counter: the cycle in which `start` is high counts as 1,
//   counting continues through the cycle in which `done` is observed high.
//
//   Also probes the per-joint divider (g_div[0].u_div): valid_in -> valid_out.
//   Sentinel: R24_A_OK
// ============================================================================
module tb_r24_adapt_latency;
    localparam integer N_UPD   = 2000;
    localparam integer CLK_NS  = 8;          // 125 MHz
    localparam [255*8-1:0] CSV = "data/adapt_latency_samples.csv";

    reg clk = 0; always #(CLK_NS/2.0) clk = ~clk;
    reg rst_n;
    reg start, adaptive_enable, prior_load;
    reg signed [31:0] tau [0:5], qe [0:5], ksn [0:5], dsn [0:5], kpr [0:5], ppr [0:5];
    wire signed [31:0] Ks_o [0:5], Ds_o [0:5], Ke_o [0:5];
    wire done;

    adaptive_impedance_v2 uut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .tau_ext_0(tau[0]), .tau_ext_1(tau[1]), .tau_ext_2(tau[2]), .tau_ext_3(tau[3]), .tau_ext_4(tau[4]), .tau_ext_5(tau[5]),
        .q_err_0(qe[0]), .q_err_1(qe[1]), .q_err_2(qe[2]), .q_err_3(qe[3]), .q_err_4(qe[4]), .q_err_5(qe[5]),
        .Ks_nom_0(ksn[0]), .Ks_nom_1(ksn[1]), .Ks_nom_2(ksn[2]), .Ks_nom_3(ksn[3]), .Ks_nom_4(ksn[4]), .Ks_nom_5(ksn[5]),
        .Ds_nom_0(dsn[0]), .Ds_nom_1(dsn[1]), .Ds_nom_2(dsn[2]), .Ds_nom_3(dsn[3]), .Ds_nom_4(dsn[4]), .Ds_nom_5(dsn[5]),
        .adaptive_enable(adaptive_enable),
        .prior_load(prior_load),
        .K_env_prior_0(kpr[0]), .K_env_prior_1(kpr[1]), .K_env_prior_2(kpr[2]), .K_env_prior_3(kpr[3]), .K_env_prior_4(kpr[4]), .K_env_prior_5(kpr[5]),
        .P_prior_0(ppr[0]), .P_prior_1(ppr[1]), .P_prior_2(ppr[2]), .P_prior_3(ppr[3]), .P_prior_4(ppr[4]), .P_prior_5(ppr[5]),
        .Ks_out_0(Ks_o[0]), .Ks_out_1(Ks_o[1]), .Ks_out_2(Ks_o[2]), .Ks_out_3(Ks_o[3]), .Ks_out_4(Ks_o[4]), .Ks_out_5(Ks_o[5]),
        .Ds_out_0(Ds_o[0]), .Ds_out_1(Ds_o[1]), .Ds_out_2(Ds_o[2]), .Ds_out_3(Ds_o[3]), .Ds_out_4(Ds_o[4]), .Ds_out_5(Ds_o[5]),
        .K_env_est_0(Ke_o[0]), .K_env_est_1(Ke_o[1]), .K_env_est_2(Ke_o[2]), .K_env_est_3(Ke_o[3]), .K_env_est_4(Ke_o[4]), .K_env_est_5(Ke_o[5]),
        .done(done)
    );

    // ---------------- cycle counter: start(=1) ... done observed (incl.) ----
    reg [15:0] cyc; reg run; reg [15:0] last_cyc;
    always @(posedge clk) begin
        if (!rst_n) begin cyc<=0; run<=0; last_cyc<=0; end
        else if (start && !run) begin run<=1; cyc<=16'd1; end
        else if (run) begin
            cyc <= cyc + 16'd1;
            if (done) begin last_cyc <= cyc + 16'd1; run <= 0; end
        end
    end

    // ---------------- divider probe: valid_in(=1) ... valid_out observed ----
    reg [15:0] dcyc; reg drun; reg [15:0] div_min, div_max; integer div_n;
    always @(posedge clk) begin
        if (!rst_n) begin dcyc<=0; drun<=0; div_min<=16'hFFFF; div_max<=0; div_n<=0; end
        else if (uut.g_div[0].u_div.valid_in && !drun) begin drun<=1; dcyc<=16'd1; end
        else if (drun) begin
            dcyc <= dcyc + 16'd1;
            if (uut.g_div[0].u_div.valid_out) begin
                drun <= 0; div_n <= div_n + 1;
                if (dcyc+16'd1 < div_min) div_min <= dcyc+16'd1;
                if (dcyc+16'd1 > div_max) div_max <= dcyc+16'd1;
            end
        end
    end

    // ---------------- random helpers ($random with explicit seed) ----------
    integer seed;
    function integer urand; input integer lo; input integer hi;   // uniform int in [lo,hi]
        integer r; begin r = $random(seed); if (r < 0) r = -r; urand = lo + (r % (hi - lo + 1)); end
    endfunction

    // ---------------- stats ----------------
    integer hist [0:4095];
    integer n, i, cat, fd, g, kenv_true, eq_lsb, ks, ds, sgn, noise;
    integer cmin, cmax, csum, cmed, acc, half;
    real    mean, var_acc, sd;
    integer cyc_rec [0:N_UPD-1];
    integer cat_rec [0:N_UPD-1];
    integer to;

    task pulse_start; begin
        @(posedge clk); #1; start = 1'b1;
        @(posedge clk); #1; start = 1'b0;
    end endtask

    task wait_done; begin
        to = 0;
        while (done !== 1'b1 && to < 5000) begin @(posedge clk); to = to + 1; end
        @(posedge clk); #1;      // let last_cyc settle
    end endtask

    initial begin
        seed = 20261014;
        rst_n = 0; start = 0; adaptive_enable = 1; prior_load = 0;
        for (i=0;i<6;i=i+1) begin tau[i]=0; qe[i]=0; ksn[i]=32'sd6553600; dsn[i]=32'sd655360; kpr[i]=32'sd78643200; ppr[i]=32'sd65536000; end
        for (i=0;i<4096;i=i+1) hist[i]=0;
        repeat (5) @(posedge clk); #1 rst_n = 1;
        repeat (5) @(posedge clk);

        fd = $fopen(CSV, "w");
        $fdisplay(fd, "n,category,adaptive_enable,eq0_q16,eq0_mm,kenv_true0,tau0_q16,ksnom0,cycles,Ks_out0,Kenv_est0");

        for (n=0; n<N_UPD; n=n+1) begin
            // periodic random prior injection (op16 path)
            if ((n % 50) == 25) begin
                for (i=0;i<6;i=i+1) begin kpr[i] = urand(100,10000) * 65536; ppr[i] = 32'sd65536000; end
                @(posedge clk); #1; prior_load = 1'b1;
                @(posedge clk); #1; prior_load = 1'b0;
                @(posedge clk);
            end
            g = urand(0,99);
            if (g < 10)      begin cat = 2; adaptive_enable = 1'b0; end   // pass-through
            else if (g < 20) begin cat = 1; adaptive_enable = 1'b1; end   // dead-zone
            else             begin cat = 0; adaptive_enable = 1'b1; end   // active
            for (i=0;i<6;i=i+1) begin
                kenv_true = urand(100,10000);                             // N/m
                if (cat==1) eq_lsb = urand(7, 64);                        // 0.1 mm .. <1 mm  (Q16.16 LSB)
                else        eq_lsb = urand(66, 1966);                     // 1 mm .. 30 mm
                sgn = urand(0,1) ? 1 : -1;
                eq_lsb = sgn * eq_lsb;
                noise = urand(-20, 20);                                   // +/- 2 %
                // tau = K_env * e_q  (N),  K_env integer N/m, e_q in Q16.16 m -> product already Q16.16
                tau[i] = (kenv_true * eq_lsb) + ((kenv_true * eq_lsb) / 1000) * noise;
                qe[i]  = eq_lsb;
                ks     = urand(100, 5000);  ksn[i] = ks * 65536;
                ds     = urand(1, 100);     dsn[i] = ds * 65536;
                if (i==0) begin cyc_rec[n] = kenv_true; end                // temp stash (overwritten below)
            end
            kenv_true = cyc_rec[n];
            pulse_start;
            wait_done;
            cyc_rec[n] = last_cyc; cat_rec[n] = cat;
            hist[last_cyc] = hist[last_cyc] + 1;
            $fdisplay(fd, "%0d,%0d,%0d,%0d,%f,%0d,%0d,%0d,%0d,%0d,%0d", n, cat, adaptive_enable,
                      qe[0], qe[0]*1000.0/65536.0, kenv_true, tau[0], ksn[0]/65536, last_cyc, Ks_o[0], Ke_o[0]);
            repeat (urand(0,3)) @(posedge clk);                            // random idle gap
        end
        $fclose(fd);

        // ---- stats ----
        cmin = 100000; cmax = 0; csum = 0;
        for (n=0;n<N_UPD;n=n+1) begin
            if (cyc_rec[n] < cmin) cmin = cyc_rec[n];
            if (cyc_rec[n] > cmax) cmax = cyc_rec[n];
            csum = csum + cyc_rec[n];
        end
        mean = csum * 1.0 / N_UPD;
        var_acc = 0.0;
        for (n=0;n<N_UPD;n=n+1) var_acc = var_acc + (cyc_rec[n]-mean)*(cyc_rec[n]-mean);
        sd = $sqrt(var_acc / N_UPD);
        acc = 0; half = N_UPD/2; cmed = -1;
        for (i=0;i<4096;i=i+1) begin acc = acc + hist[i]; if (cmed < 0 && acc > half) cmed = i; end

        $display("=============================================================");
        $display("R2.4-A  adaptive_impedance_v2 (deployed) update latency, N=%0d", N_UPD);
        $display("  start->done cycles: min=%0d  median=%0d  max=%0d  mean=%f  sd=%f", cmin, cmed, cmax, mean, sd);
        $display("  histogram (non-empty bins):");
        for (i=0;i<4096;i=i+1) if (hist[i]>0) $display("    %0d cycles : %0d updates", i, hist[i]);
        $display("  per-category counts: active=%0d deadzone=%0d enable0=%0d",
                 hist_cat(0), hist_cat(1), hist_cat(2));
        $display("  pipe_div32 valid_in->valid_out: n=%0d min=%0d max=%0d", div_n, div_min, div_max);
        $display("  csv: %0s", CSV);
        $display("R24_A_SUMMARY min=%0d median=%0d max=%0d sd=%f divmin=%0d divmax=%0d", cmin, cmed, cmax, sd, div_min, div_max);
        if (cmax > 0 && div_n == 3*N_UPD) $display("R24_A_OK"); else $display("R24_A_FAIL");
        $finish;
    end

    function integer hist_cat; input integer c; integer k; begin
        hist_cat = 0; for (k=0;k<N_UPD;k=k+1) if (cat_rec[k]==c) hist_cat = hist_cat + 1;
    end endfunction

    initial begin #200000000; $display("TIMEOUT"); $display("R24_A_FAIL"); $finish; end
endmodule
