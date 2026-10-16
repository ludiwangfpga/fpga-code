`timescale 1ns/1ps
// ============================================================================
// tb_r24_udp_serial_vs_bypass.v -- R2.4 experiment B, cross-check on the REAL
//   deployed command FSM (rtl/top/udp_fk_cmd.v of this repository, the file
//   compiled into board_test/bitstream/top_udp_add.bit, which contains the op18 on-chip
//   cycle counter whose min/max registers the paper reports as 549/549).
//
//   Driver / udp_tx receive model / probes are taken from
//   sim/tb_r2_1_fwd_cycles.v.  Per sample (N_SMP random beats):
//     op15 (random e_q / Ks_nom / Ds_nom, en=1, random iq_actual)  -> T_adapt
//     op2  (random joint angles)                                    -> fk seg
//     op10 (random x_des / K / D / xdot)                            -> imp+tiq
//     op18 read-back                                                -> cyc_last/min/max/runs (bypass, on-chip)
//   Serial-equivalent forward latency per beat = cyc_last + T_adapt, because
//   a serial command FSM would insert the S_ADAPT_WAIT segment between the
//   op10 dispatch cycle and imp_start with the same done->start handshake
//   (0 idle cycles) that the counter already uses between imp_done and
//   tiq_start (measured gap = 0 in tb_r2_1_fwd_cycles.v).
//   Sentinel: R24_C_OK
// ============================================================================
module tb_r24_udp_serial_vs_bypass;
    localparam integer N_SMP = 1000;
    localparam [255*8-1:0] CSV = "data/udp_samples.csv";

    reg clk, rst;
    reg  [7:0] s_udp_axis_data;
    reg        s_udp_axis_valid, s_udp_axis_last, s_udp_axis_error;
    wire [7:0] m_udp_axis_data;
    wire       m_udp_axis_valid, m_udp_axis_last;
    wire [10:0] cfg_udp_len;
    reg        m_udp_axis_ready;
    reg [9:0]         tb_duty [0:17];
    reg signed [15:0] tb_iq   [0:5];

    udp_fk_cmd uut (
        .clk(clk), .rst(rst),
        .s_udp_axis_data(s_udp_axis_data), .s_udp_axis_valid(s_udp_axis_valid),
        .s_udp_axis_last(s_udp_axis_last), .s_udp_axis_error(s_udp_axis_error),
        .m_udp_axis_data(m_udp_axis_data), .m_udp_axis_valid(m_udp_axis_valid),
        .m_udp_axis_last(m_udp_axis_last), .m_udp_axis_ready(m_udp_axis_ready),
        .cfg_udp_len(cfg_udp_len),
        .iq_out0(), .iq_out1(), .iq_out2(), .iq_out3(), .iq_out4(), .iq_out5(),
        .duty_a_0(tb_duty[0]),  .duty_b_0(tb_duty[1]),  .duty_c_0(tb_duty[2]),
        .duty_a_1(tb_duty[3]),  .duty_b_1(tb_duty[4]),  .duty_c_1(tb_duty[5]),
        .duty_a_2(tb_duty[6]),  .duty_b_2(tb_duty[7]),  .duty_c_2(tb_duty[8]),
        .duty_a_3(tb_duty[9]),  .duty_b_3(tb_duty[10]), .duty_c_3(tb_duty[11]),
        .duty_a_4(tb_duty[12]), .duty_b_4(tb_duty[13]), .duty_c_4(tb_duty[14]),
        .duty_a_5(tb_duty[15]), .duty_b_5(tb_duty[16]), .duty_c_5(tb_duty[17]),
        .phi_0(), .phi_1(), .phi_2(), .phi_3(), .phi_4(), .phi_5(),
        .adc_a_0(), .adc_b_0(), .adc_c_0(), .adc_a_1(), .adc_b_1(), .adc_c_1(),
        .adc_a_2(), .adc_b_2(), .adc_c_2(), .adc_a_3(), .adc_b_3(), .adc_c_3(),
        .adc_a_4(), .adc_b_4(), .adc_c_4(), .adc_a_5(), .adc_b_5(), .adc_c_5(),
        .iq_act_0(tb_iq[0]), .iq_act_1(tb_iq[1]), .iq_act_2(tb_iq[2]),
        .iq_act_3(tb_iq[3]), .iq_act_4(tb_iq[4]), .iq_act_5(tb_iq[5])
    );

    initial begin clk = 0; forever #4 clk = ~clk; end   // 125 MHz

    // ---------------- udp_tx contract receive model (from sim/tb_r2_1_fwd_cycles.v) ----------
    localparam M_IDLE=2'd0, M_WRITE=2'd1, M_DONE=2'd2;
    reg [1:0]  mst; reg [10:0] mlen, mwidx; reg [7:0] mbuf [0:255]; reg pkt_sent;
    always @(*) m_udp_axis_ready = (mst==M_IDLE) || (mst==M_WRITE);
    always @(posedge clk) begin
        if (rst) begin mst<=M_IDLE; mwidx<=0; pkt_sent<=0; mlen<=0; end
        else begin
            pkt_sent <= 0;
            case (mst)
            M_IDLE:  if (m_udp_axis_valid) begin mlen<=cfg_udp_len; mbuf[0]<=m_udp_axis_data; mwidx<=1; mst<=M_WRITE; end
            M_WRITE: if (m_udp_axis_valid) begin mbuf[mwidx[7:0]]<=m_udp_axis_data; mwidx<=mwidx+1; if (m_udp_axis_last) mst<=M_DONE; end
            M_DONE:  begin pkt_sent<=1; mwidx<=0; mst<=M_IDLE; end
            default: mst<=M_IDLE;
            endcase
        end
    end

    // ---------------- segment probes (same semantic as sim/tb_r2_1_fwd_cycles.v) ----------
    reg [15:0] fk_cnt, imp_cnt, tiq_cnt, adapt_cnt, chain_cnt;
    reg fk_run, imp_run, tiq_run, adapt_run, chain_run, fk_sd, imp_sd, tiq_sd, adapt_sd;
    reg [15:0] last_fk, last_imp, last_tiq, last_adapt, last_chain;
    always @(posedge clk) begin
        if (rst) begin
            fk_cnt<=0; imp_cnt<=0; tiq_cnt<=0; adapt_cnt<=0; chain_cnt<=0;
            fk_run<=0; imp_run<=0; tiq_run<=0; adapt_run<=0; chain_run<=0;
            fk_sd<=0; imp_sd<=0; tiq_sd<=0; adapt_sd<=0;
            last_fk<=0; last_imp<=0; last_tiq<=0; last_adapt<=0; last_chain<=0;
        end else begin
            fk_sd<=uut.fk_start; imp_sd<=uut.imp_start; tiq_sd<=uut.tiq_start; adapt_sd<=uut.adapt_start;
            if (uut.fk_start && !fk_sd) begin fk_run<=1; fk_cnt<=1; end
            else if (fk_run) begin fk_cnt<=fk_cnt+1; if (uut.fk_done) begin last_fk<=fk_cnt+1; fk_run<=0; end end
            if (uut.imp_start && !imp_sd) begin imp_run<=1; imp_cnt<=1; chain_run<=1; chain_cnt<=1; end
            else if (imp_run) begin imp_cnt<=imp_cnt+1; if (uut.imp_done) begin last_imp<=imp_cnt+1; imp_run<=0; end end
            if (uut.tiq_start && !tiq_sd) begin tiq_run<=1; tiq_cnt<=1; end
            else if (tiq_run) begin tiq_cnt<=tiq_cnt+1; if (uut.tiq_done) begin last_tiq<=tiq_cnt+1; tiq_run<=0; end end
            if (chain_run) begin chain_cnt<=chain_cnt+1; if (uut.tiq_done) begin last_chain<=chain_cnt+1; chain_run<=0; end end
            if (uut.adapt_start && !adapt_sd) begin adapt_run<=1; adapt_cnt<=1; end
            else if (adapt_run) begin adapt_cnt<=adapt_cnt+1; if (uut.adapt_done) begin last_adapt<=adapt_cnt+1; adapt_run<=0; end end
        end
    end

    // ---------------- frame driver ----------------
    reg [7:0] fr [0:127]; integer frn, i, j, guard; reg t_to;
    task drive_frame; begin
        @(posedge clk);
        for (j=0; j<frn; j=j+1) begin
            s_udp_axis_data <= fr[j]; s_udp_axis_valid <= 1'b1; s_udp_axis_last <= (j==frn-1); s_udp_axis_error <= 1'b0;
            @(posedge clk);
        end
        s_udp_axis_valid <= 0; s_udp_axis_last <= 0; s_udp_axis_error <= 0;
    end endtask
    task await_pkt; output to; begin
        to = 0; guard = 0;
        while (pkt_sent !== 1'b1) begin @(posedge clk); guard = guard + 1; if (guard > 20000) begin to = 1; disable await_pkt; end end
    end endtask

    reg [31:0] qv [0:5], XDv [0:5], Kv [0:5], Dv [0:5], XVv [0:5], QEv [0:5], KNv [0:5], DNv [0:5];
    task put32; input integer off; input [31:0] w; begin fr[off]=w[7:0]; fr[off+1]=w[15:8]; fr[off+2]=w[23:16]; fr[off+3]=w[31:24]; end endtask
    task build_op2;  begin fr[0]=8'd2;  fr[1]=0;fr[2]=0;fr[3]=0;fr[4]=0; fr[5]=8'd24; fr[6]=0; for (i=0;i<6;i=i+1) put32(7+4*i, qv[i]); frn=31; end endtask
    task build_op10; begin fr[0]=8'd10; fr[1]=0;fr[2]=0;fr[3]=0;fr[4]=0; fr[5]=8'd96; fr[6]=0;
        for (i=0;i<6;i=i+1) begin put32(7+4*i, XDv[i]); put32(31+4*i, Kv[i]); put32(55+4*i, Dv[i]); put32(79+4*i, XVv[i]); end frn=103; end endtask
    task build_op15; input en; begin fr[0]=8'd15; fr[1]=0;fr[2]=0;fr[3]=0;fr[4]=0; fr[5]=8'd73; fr[6]=0; fr[7]=en?8'd1:8'd0;
        for (i=0;i<6;i=i+1) begin put32(8+4*i, QEv[i]); put32(32+4*i, KNv[i]); put32(56+4*i, DNv[i]); end frn=80; end endtask
    task build_op16; input [31:0] kenv; input [31:0] pcov; begin fr[0]=8'd16; fr[1]=0;fr[2]=0;fr[3]=0;fr[4]=0; fr[5]=8'd48; fr[6]=0;
        for (i=0;i<6;i=i+1) begin put32(7+4*i, kenv); put32(31+4*i, pcov); end frn=55; end endtask
    task build_op18; begin fr[0]=8'd18; fr[1]=0;fr[2]=0;fr[3]=0;fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7; end endtask
    function signed [31:0] mword; input integer n; mword = {mbuf[4*n+3], mbuf[4*n+2], mbuf[4*n+1], mbuf[4*n]}; endfunction

    integer seed;
    function integer urand; input integer lo; input integer hi;
        integer r; begin r = $random(seed); if (r < 0) r = -r; urand = lo + (r % (hi - lo + 1)); end
    endfunction

    integer n, fd, g, kenv, eql, sgn, errors;
    integer recA [0:N_SMP-1], recF [0:N_SMP-1], recS [0:N_SMP-1];
    integer cyc_last, cyc_min, cyc_max, cyc_runs, magic;
    integer minA,maxA,minF,maxF,minS,maxS,medS,acc; real meanS,varS,sdS, meanA;
    integer hist [0:4095];
    integer ti;

    initial begin
        seed = 4242; errors = 0;
        rst = 1; s_udp_axis_data=0; s_udp_axis_valid=0; s_udp_axis_last=0; s_udp_axis_error=0;
        for (ti=0;ti<18;ti=ti+1) tb_duty[ti]=10'd512;
        for (ti=0;ti<6;ti=ti+1)  tb_iq[ti]=16'sd200;
        for (ti=0;ti<4096;ti=ti+1) hist[ti]=0;
        repeat (10) @(posedge clk); rst = 0; repeat (10) @(posedge clk);

        fd = $fopen(CSV, "w");
        $fdisplay(fd, "n,adapt_cycles,fk_cycles,imp_cycles,tiq_cycles,onchip_cyc_last,onchip_cyc_min,onchip_cyc_max,onchip_runs,serial_equiv_cycles,eq0_q16,Ksnom0");

        for (n=0; n<N_SMP; n=n+1) begin
            if ((n % 100) == 50) begin build_op16(urand(100,10000)*65536, 32'sd65536000); drive_frame; await_pkt(t_to); end
            g = urand(0,99);
            for (i=0;i<6;i=i+1) begin
                qv[i]  = urand(-102900,102900);
                XDv[i] = urand(-19660,19660); Kv[i] = urand(100,5000)*65536; Dv[i] = urand(1,100)*65536; XVv[i] = urand(-6553,6553);
                kenv   = urand(100,10000);
                eql    = (g < 10) ? urand(7,64) : urand(66,1966);
                sgn    = urand(0,1) ? 1 : -1; QEv[i] = sgn*eql;
                KNv[i] = urand(100,5000)*65536; DNv[i] = urand(1,100)*65536;
                tb_iq[i] = urand(-500,500);
            end
            // 1) adaptation beat (op15, en=1 except 10 % en=0)
            build_op15((g >= 90) ? 1'b0 : 1'b1); drive_frame; await_pkt(t_to);
            if (t_to) begin errors=errors+1; $display("TIMEOUT op15 n=%0d", n); end
            // 2) forward chain (op2 + op10) -> on-chip counter window
            build_op2;  drive_frame; await_pkt(t_to); if (t_to) begin errors=errors+1; $display("TIMEOUT op2 n=%0d", n); end
            build_op10; drive_frame; await_pkt(t_to); if (t_to) begin errors=errors+1; $display("TIMEOUT op10 n=%0d", n); end
            // 3) read on-chip counter
            build_op18; drive_frame; await_pkt(t_to); if (t_to) begin errors=errors+1; $display("TIMEOUT op18 n=%0d", n); end
            @(posedge clk);
            cyc_last = mword(0); cyc_min = mword(1); cyc_max = mword(2); cyc_runs = mword(3); magic = mword(4);
            if (magic !== 32'hE2E2E2E2) begin errors=errors+1; $display("BAD MAGIC n=%0d %08x", n, magic); end
            recA[n] = last_adapt; recF[n] = cyc_last; recS[n] = cyc_last + last_adapt;
            hist[recS[n]] = hist[recS[n]] + 1;
            $fdisplay(fd, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d", n, last_adapt, last_fk, last_imp, last_tiq,
                      cyc_last, cyc_min, cyc_max, cyc_runs, recS[n], $signed(QEv[0]), KNv[0]/65536);
        end
        $fclose(fd);

        minA=100000;maxA=0;minF=100000;maxF=0;minS=100000;maxS=0;meanS=0;meanA=0;
        for (n=0;n<N_SMP;n=n+1) begin
            if (recA[n]<minA) minA=recA[n]; if (recA[n]>maxA) maxA=recA[n];
            if (recF[n]<minF) minF=recF[n]; if (recF[n]>maxF) maxF=recF[n];
            if (recS[n]<minS) minS=recS[n]; if (recS[n]>maxS) maxS=recS[n];
            meanS = meanS + recS[n]; meanA = meanA + recA[n];
        end
        meanS = meanS/N_SMP; meanA = meanA/N_SMP; varS=0;
        for (n=0;n<N_SMP;n=n+1) varS = varS + (recS[n]-meanS)*(recS[n]-meanS);
        sdS = $sqrt(varS/N_SMP);
        acc=0; medS=-1; for (ti=0;ti<4096;ti=ti+1) begin acc=acc+hist[ti]; if (medS<0 && acc>N_SMP/2) medS=ti; end

        $display("=============================================================");
        $display("R2.4-C  deployed udp_fk_cmd.v (op18 on-chip counter), N=%0d beats", N_SMP);
        $display("  on-chip forward window (bypass)  : last min=%0d max=%0d ; counter min=%0d max=%0d runs=%0d", minF, maxF, cyc_min, cyc_max, cyc_runs);
        $display("  adaptation op15 start->done      : min=%0d max=%0d mean=%f", minA, maxA, meanA);
        $display("  serial-equivalent (fwd + adapt)  : min=%0d median=%0d max=%0d mean=%f sd=%f", minS, medS, maxS, meanS, sdS);
        $display("R24_C_SUMMARY fwd=%0d/%0d chipmin=%0d chipmax=%0d adapt=%0d/%0d ser=%0d/%0d/%0d", minF, maxF, cyc_min, cyc_max, minA, maxA, minS, medS, maxS);
        if (errors==0 && cyc_min==cyc_max && cyc_runs==N_SMP) $display("R24_C_OK"); else $display("R24_C_FAIL errors=%0d", errors);
        $finish;
    end
    initial begin #400000000; $display("TIMEOUT"); $display("R24_C_FAIL"); $finish; end
endmodule
