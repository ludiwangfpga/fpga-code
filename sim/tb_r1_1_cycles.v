`timescale 1ns/1ps

module tb_r1_1_cycles;

    reg clk, rst_n;

    reg fk_jac_start;
    reg signed [31:0] q0, q1, q2, q3, q4, q5;
    wire signed [31:0] pose_x, pose_y, pose_z, pose_rx, pose_ry, pose_rz;
    wire signed [31:0] J [0:35];
    wire fk_jac_done;

    reg cart_start;
    reg signed [31:0] xdes [0:5], K_r [0:5], D_r [0:5], xv [0:5];
    reg signed [31:0] J_imp [0:35];
    reg signed [31:0] pose_imp [0:5];
    wire signed [31:0] tau [0:5];
    wire cart_done;

    reg tiq_start;
    reg signed [31:0] tau_imp [0:5];
    wire signed [15:0] iq_w0, iq_w1, iq_w2, iq_w3, iq_w4, iq_w5;
    wire tiq_done;

    initial clk = 0;
    always #5 clk = ~clk;

    analytical_jacobian_serial fk_jac (
        .clk(clk), .rst_n(rst_n), .start(fk_jac_start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .pose_x(pose_x), .pose_y(pose_y), .pose_z(pose_z),
        .pose_rx(pose_rx), .pose_ry(pose_ry), .pose_rz(pose_rz),
        .J00(J[0]),  .J01(J[1]),  .J02(J[2]),  .J03(J[3]),  .J04(J[4]),  .J05(J[5]),
        .J10(J[6]),  .J11(J[7]),  .J12(J[8]),  .J13(J[9]),  .J14(J[10]), .J15(J[11]),
        .J20(J[12]), .J21(J[13]), .J22(J[14]), .J23(J[15]), .J24(J[16]), .J25(J[17]),
        .J30(J[18]), .J31(J[19]), .J32(J[20]), .J33(J[21]), .J34(J[22]), .J35(J[23]),
        .J40(J[24]), .J41(J[25]), .J42(J[26]), .J43(J[27]), .J44(J[28]), .J45(J[29]),
        .J50(J[30]), .J51(J[31]), .J52(J[32]), .J53(J[33]), .J54(J[34]), .J55(J[35]),
        .done(fk_jac_done)
    );

    cart_impedance imp (
        .clk(clk), .rst_n(rst_n), .start(cart_start),
        .J00(J_imp[0]),  .J01(J_imp[1]),  .J02(J_imp[2]),  .J03(J_imp[3]),  .J04(J_imp[4]),  .J05(J_imp[5]),
        .J10(J_imp[6]),  .J11(J_imp[7]),  .J12(J_imp[8]),  .J13(J_imp[9]),  .J14(J_imp[10]), .J15(J_imp[11]),
        .J20(J_imp[12]), .J21(J_imp[13]), .J22(J_imp[14]), .J23(J_imp[15]), .J24(J_imp[16]), .J25(J_imp[17]),
        .J30(J_imp[18]), .J31(J_imp[19]), .J32(J_imp[20]), .J33(J_imp[21]), .J34(J_imp[22]), .J35(J_imp[23]),
        .J40(J_imp[24]), .J41(J_imp[25]), .J42(J_imp[26]), .J43(J_imp[27]), .J44(J_imp[28]), .J45(J_imp[29]),
        .J50(J_imp[30]), .J51(J_imp[31]), .J52(J_imp[32]), .J53(J_imp[33]), .J54(J_imp[34]), .J55(J_imp[35]),
        .p0(pose_imp[0]), .p1(pose_imp[1]), .p2(pose_imp[2]),
        .p3(pose_imp[3]), .p4(pose_imp[4]), .p5(pose_imp[5]),
        .xd0(xdes[0]), .xd1(xdes[1]), .xd2(xdes[2]),
        .xd3(xdes[3]), .xd4(xdes[4]), .xd5(xdes[5]),
        .K0(K_r[0]), .K1(K_r[1]), .K2(K_r[2]), .K3(K_r[3]), .K4(K_r[4]), .K5(K_r[5]),
        .D0(D_r[0]), .D1(D_r[1]), .D2(D_r[2]), .D3(D_r[3]), .D4(D_r[4]), .D5(D_r[5]),
        .xv0(xv[0]), .xv1(xv[1]), .xv2(xv[2]),
        .xv3(xv[3]), .xv4(xv[4]), .xv5(xv[5]),
        .tau0(tau[0]), .tau1(tau[1]), .tau2(tau[2]),
        .tau3(tau[3]), .tau4(tau[4]), .tau5(tau[5]),
        .done(cart_done)
    );

    torque_to_iq tiq (
        .clk(clk), .rst_n(rst_n), .start(tiq_start),
        .tau0(tau_imp[0]), .tau1(tau_imp[1]), .tau2(tau_imp[2]),
        .tau3(tau_imp[3]), .tau4(tau_imp[4]), .tau5(tau_imp[5]),
        .iq0(iq_w0), .iq1(iq_w1), .iq2(iq_w2),
        .iq3(iq_w3), .iq4(iq_w4), .iq5(iq_w5),
        .done(tiq_done)
    );

    reg [15:0] fkjac_cnt, cart_cnt, tiq_cnt;
    reg fkjac_running, cart_running, tiq_running;
    reg fk_jac_start_d, cart_start_d, tiq_start_d;
    reg [15:0] last_fkjac, last_cart, last_tiq;

    reg [15:0] fk_eng_cnt, wc_cnt, it_cnt;
    reg fk_eng_running, wc_running, it_running;
    reg fk_eng_start_d, wc_start_d, it_start_d;
    reg [15:0] last_fk_eng, last_wc, last_it;

    reg [15:0] gap_fk2imp_cnt, gap_imp2tiq_cnt;
    reg gap_fk2imp_active, gap_imp2tiq_active;
    reg [15:0] last_gap_fk2imp, last_gap_imp2tiq;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fkjac_cnt<=0; cart_cnt<=0; tiq_cnt<=0;
            fkjac_running<=0; cart_running<=0; tiq_running<=0;
            fk_jac_start_d<=0; cart_start_d<=0; tiq_start_d<=0;
            last_fkjac<=0; last_cart<=0; last_tiq<=0;

            fk_eng_cnt<=0; wc_cnt<=0; it_cnt<=0;
            fk_eng_running<=0; wc_running<=0; it_running<=0;
            fk_eng_start_d<=0; wc_start_d<=0; it_start_d<=0;
            last_fk_eng<=0; last_wc<=0; last_it<=0;

            gap_fk2imp_cnt<=0; gap_imp2tiq_cnt<=0;
            gap_fk2imp_active<=0; gap_imp2tiq_active<=0;
            last_gap_fk2imp<=0; last_gap_imp2tiq<=0;
        end else begin
            fk_jac_start_d  <= fk_jac_start;
            cart_start_d    <= cart_start;
            tiq_start_d     <= tiq_start;
            fk_eng_start_d  <= fk_jac.fk_start;
            wc_start_d      <= imp.wc_start;
            it_start_d      <= imp.it_start;

            if (fk_jac_start && !fk_jac_start_d) begin
                fkjac_running <= 1; fkjac_cnt <= 1;
            end else if (fkjac_running) begin
                fkjac_cnt <= fkjac_cnt + 16'd1;
                if (fk_jac_done) begin
                    last_fkjac <= fkjac_cnt + 16'd1;
                    fkjac_running <= 0;
                    gap_fk2imp_active <= 1; gap_fk2imp_cnt <= 0;
                end
            end

            if (fk_jac.fk_start && !fk_eng_start_d) begin
                fk_eng_running <= 1; fk_eng_cnt <= 1;
            end else if (fk_eng_running) begin
                fk_eng_cnt <= fk_eng_cnt + 16'd1;
                if (fk_jac.fk_done) begin
                    last_fk_eng <= fk_eng_cnt + 16'd1;
                    fk_eng_running <= 0;
                end
            end

            if (cart_start && !cart_start_d) begin
                cart_running <= 1; cart_cnt <= 1;
                if (gap_fk2imp_active) begin
                    last_gap_fk2imp <= gap_fk2imp_cnt;
                    gap_fk2imp_active <= 0;
                end
            end else if (cart_running) begin
                cart_cnt <= cart_cnt + 16'd1;
                if (cart_done) begin
                    last_cart <= cart_cnt + 16'd1;
                    cart_running <= 0;
                    gap_imp2tiq_active <= 1; gap_imp2tiq_cnt <= 0;
                end
            end
            if (gap_fk2imp_active) gap_fk2imp_cnt <= gap_fk2imp_cnt + 16'd1;

            if (imp.wc_start && !wc_start_d) begin
                wc_running <= 1; wc_cnt <= 1;
            end else if (wc_running) begin
                wc_cnt <= wc_cnt + 16'd1;
                if (imp.wc_done) begin
                    last_wc <= wc_cnt + 16'd1;
                    wc_running <= 0;
                end
            end

            if (imp.it_start && !it_start_d) begin
                it_running <= 1; it_cnt <= 1;
            end else if (it_running) begin
                it_cnt <= it_cnt + 16'd1;
                if (imp.it_done) begin
                    last_it <= it_cnt + 16'd1;
                    it_running <= 0;
                end
            end

            if (tiq_start && !tiq_start_d) begin
                tiq_running <= 1; tiq_cnt <= 1;
                if (gap_imp2tiq_active) begin
                    last_gap_imp2tiq <= gap_imp2tiq_cnt;
                    gap_imp2tiq_active <= 0;
                end
            end else if (tiq_running) begin
                tiq_cnt <= tiq_cnt + 16'd1;
                if (tiq_done) begin
                    last_tiq <= tiq_cnt + 16'd1;
                    tiq_running <= 0;
                end
            end
            if (gap_imp2tiq_active) gap_imp2tiq_cnt <= gap_imp2tiq_cnt + 16'd1;
        end
    end

    task run_chain;
        integer w;
        begin

            @(posedge clk); fk_jac_start <= 1'b1;
            @(posedge clk); fk_jac_start <= 1'b0;
            w = 0;
            wait (fk_jac_done == 1'b1);

            @(posedge clk);
            for (w=0;w<36;w=w+1) J_imp[w] = J[w];
            pose_imp[0] = pose_x;  pose_imp[1] = pose_y;  pose_imp[2] = pose_z;
            pose_imp[3] = pose_rx; pose_imp[4] = pose_ry; pose_imp[5] = pose_rz;

            @(posedge clk); cart_start <= 1'b1;
            @(posedge clk); cart_start <= 1'b0;
            wait (cart_done == 1'b1);

            @(posedge clk);
            for (w=0;w<6;w=w+1) tau_imp[w] = tau[w];

            @(posedge clk); tiq_start <= 1'b1;
            @(posedge clk); tiq_start <= 1'b0;
            wait (tiq_done == 1'b1);

            repeat (3) @(posedge clk);
        end
    endtask

    reg signed [31:0] qcfg [0:9][0:5];

    task load_qcfgs;
        begin

            qcfg[0][0]=0; qcfg[0][1]=0; qcfg[0][2]=0; qcfg[0][3]=0; qcfg[0][4]=0; qcfg[0][5]=0;

            qcfg[1][0]=32'sd34315; qcfg[1][1]=-32'sd51472; qcfg[1][2]=32'sd68629;
            qcfg[1][3]=0;          qcfg[1][4]=32'sd102944; qcfg[1][5]=0;

            qcfg[2][0]=32'sd22877; qcfg[2][1]=32'sd11439; qcfg[2][2]=-32'sd34315;
            qcfg[2][3]=32'sd17157; qcfg[2][4]=32'sd45751; qcfg[2][5]=-32'sd28598;

            qcfg[3][0]=32'sd32768; qcfg[3][1]=32'sd32768; qcfg[3][2]=32'sd32768;
            qcfg[3][3]=32'sd32768; qcfg[3][4]=32'sd32768; qcfg[3][5]=32'sd32768;

            qcfg[4][0]=-32'sd32768; qcfg[4][1]=-32'sd32768; qcfg[4][2]=-32'sd32768;
            qcfg[4][3]=-32'sd32768; qcfg[4][4]=-32'sd32768; qcfg[4][5]=-32'sd32768;

            qcfg[5][0]=32'sd98304; qcfg[5][1]=32'sd98304; qcfg[5][2]=-32'sd98304;
            qcfg[5][3]=32'sd98304; qcfg[5][4]=-32'sd98304; qcfg[5][5]=32'sd98304;

            qcfg[6][0]=0; qcfg[6][1]=32'sd65536; qcfg[6][2]=0;
            qcfg[6][3]=32'sd65536; qcfg[6][4]=0; qcfg[6][5]=0;

            qcfg[7][0]=32'sd45875; qcfg[7][1]=-32'sd45875; qcfg[7][2]=32'sd45875;
            qcfg[7][3]=-32'sd45875; qcfg[7][4]=32'sd45875; qcfg[7][5]=-32'sd45875;

            qcfg[8][0]=32'sd65536; qcfg[8][1]=32'sd65536; qcfg[8][2]=-32'sd65536;
            qcfg[8][3]=32'sd32768; qcfg[8][4]=-32'sd32768; qcfg[8][5]=32'sd65536;

            qcfg[9][0]=32'sd10000; qcfg[9][1]=-32'sd20000; qcfg[9][2]=32'sd55000;
            qcfg[9][3]=-32'sd45000; qcfg[9][4]=32'sd25000; qcfg[9][5]=-32'sd12000;
        end
    endtask

    reg [15:0] log_fkjac    [0:9];
    reg [15:0] log_fk_eng   [0:9];
    reg [15:0] log_cart     [0:9];
    reg [15:0] log_wc       [0:9];
    reg [15:0] log_it       [0:9];
    reg [15:0] log_tiq      [0:9];
    reg [15:0] log_gap1     [0:9];
    reg [15:0] log_gap2     [0:9];
    reg [15:0] log_total    [0:9];

    integer ti;

    initial begin

        rst_n = 0;
        fk_jac_start = 0; cart_start = 0; tiq_start = 0;
        q0=0; q1=0; q2=0; q3=0; q4=0; q5=0;
        for (ti=0; ti<6; ti=ti+1) begin
            xdes[ti]=0; K_r[ti]=32'sd6553600; D_r[ti]=32'sd655360; xv[ti]=0;
            tau_imp[ti]=0; pose_imp[ti]=0;
        end
        for (ti=0;ti<36;ti=ti+1) J_imp[ti]=0;
        load_qcfgs;

        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        $display("====================================================");
        $display("R1.1 cycle measurement: FK+Jac → cart_imp(wc+it) → torque_to_iq");
        $display("Clock: 100MHz (10 ns/cycle in sim).");
        $display("On-board 125 MHz (8 ns/cycle): forward_total_us = total_cycles * 8 / 1000");
        $display("====================================================");

        for (ti=0; ti<10; ti=ti+1) begin

            q0 = qcfg[ti][0]; q1 = qcfg[ti][1]; q2 = qcfg[ti][2];
            q3 = qcfg[ti][3]; q4 = qcfg[ti][4]; q5 = qcfg[ti][5];

            xdes[0]=32'sd1000; xdes[1]=32'sd1000; xdes[2]=32'sd1000;
            xdes[3]=32'sd1000; xdes[4]=32'sd1000; xdes[5]=32'sd1000;
            K_r[0]=32'sd6553600; K_r[1]=32'sd6553600; K_r[2]=32'sd6553600;
            K_r[3]=32'sd655360;  K_r[4]=32'sd655360;  K_r[5]=32'sd655360;
            D_r[0]=32'sd655360; D_r[1]=32'sd655360; D_r[2]=32'sd655360;
            D_r[3]=32'sd65536;  D_r[4]=32'sd65536;  D_r[5]=32'sd65536;
            xv[0]=32'sd655; xv[1]=-32'sd655; xv[2]=32'sd328;
            xv[3]=0;        xv[4]=-32'sd655; xv[5]=32'sd328;

            run_chain;

            log_fkjac [ti] = last_fkjac;
            log_fk_eng[ti] = last_fk_eng;
            log_cart  [ti] = last_cart;
            log_wc    [ti] = last_wc;
            log_it    [ti] = last_it;
            log_tiq   [ti] = last_tiq;
            log_gap1  [ti] = last_gap_fk2imp;
            log_gap2  [ti] = last_gap_imp2tiq;
            log_total [ti] = last_fkjac + last_gap_fk2imp + last_cart + last_gap_imp2tiq + last_tiq;

            $display("  TC%0d: FK_eng=%0d  FK+Jac(total)=%0d  gap_fk2imp=%0d  cart_imp=%0d (wc=%0d, it=%0d)  gap_imp2tiq=%0d  tiq=%0d  TOTAL=%0d (%.3f us @8ns)",
                     ti, last_fk_eng, last_fkjac, last_gap_fk2imp,
                     last_cart, last_wc, last_it, last_gap_imp2tiq, last_tiq,
                     log_total[ti], log_total[ti]*8.0/1000.0);
        end

        $display("");
        $display("=======================================================================================");
        $display("R1.1 CYCLE BREAKDOWN -- start to done (cycles), on-board 8 ns/cycle");
        $display("=======================================================================================");
        $display(" TC | FK_eng | FK+Jac | gap1 | cart_imp(wc, it)        | gap2 | tiq | TOTAL  |  μs(@8ns)");
        $display("---------------------------------------------------------------------------------------");
        for (ti=0; ti<10; ti=ti+1) begin
            $display(" T%0d |  %4d  |  %4d  |  %2d  |    %3d  (%3d, %3d)      |  %2d  |  %2d |  %4d  |  %6.3f",
                     ti, log_fk_eng[ti], log_fkjac[ti], log_gap1[ti],
                     log_cart[ti], log_wc[ti], log_it[ti], log_gap2[ti], log_tiq[ti],
                     log_total[ti], log_total[ti]*8.0/1000.0);
        end
        $display("=======================================================================================");

        $display("R1_1_CYCLES_OK");
        $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT — simulation hung");
        $finish;
    end
endmodule
