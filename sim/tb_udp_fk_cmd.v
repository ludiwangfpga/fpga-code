`timescale 1ns/1ps
// tb_udp_fk_cmd — M2.2: FK 完整位姿校验（位置+欧拉角）+ udp_tx 契约接收模型。
module tb_udp_fk_cmd;
    localparam [7:0] OP_WRITE=8'd2, OP_READ=8'd8, OP_START=8'd3, OP_NOP7=8'd7;
    localparam [7:0] ACK_BYTE=8'hAA;

    reg clk, rst;
    reg  [7:0] s_udp_axis_data; reg s_udp_axis_valid, s_udp_axis_last, s_udp_axis_error;
    wire [7:0] m_udp_axis_data; wire m_udp_axis_valid, m_udp_axis_last;
    wire [10:0] cfg_udp_len;
    reg  m_udp_axis_ready;

    integer test_cnt, pass_cnt, fail, j, guard, b;
    reg [7:0] fr [0:127]; integer frn;
    reg [31:0] qv [0:5];
    reg [7:0]  wpay [0:23];   // 期望回环的 24 字节
    reg t_to;
    reg [9:0] tb_duty [0:17]; // op13 驱动用: 18 个占空比 (6电机×3相)
    reg signed [15:0] tb_iq [0:5]; // op15 驱动用: 实测 iq_actual (片上算 τ_ext)

    udp_fk_cmd dut(.clk(clk),.rst(rst),
        .s_udp_axis_data(s_udp_axis_data),.s_udp_axis_valid(s_udp_axis_valid),
        .s_udp_axis_last(s_udp_axis_last),.s_udp_axis_error(s_udp_axis_error),
        .m_udp_axis_data(m_udp_axis_data),.m_udp_axis_valid(m_udp_axis_valid),
        .m_udp_axis_last(m_udp_axis_last),.m_udp_axis_ready(m_udp_axis_ready),
        .cfg_udp_len(cfg_udp_len),
        .iq_out0(),.iq_out1(),.iq_out2(),.iq_out3(),.iq_out4(),.iq_out5(),
        .duty_a_0(tb_duty[0]), .duty_b_0(tb_duty[1]), .duty_c_0(tb_duty[2]),
        .duty_a_1(tb_duty[3]), .duty_b_1(tb_duty[4]), .duty_c_1(tb_duty[5]),
        .duty_a_2(tb_duty[6]), .duty_b_2(tb_duty[7]), .duty_c_2(tb_duty[8]),
        .duty_a_3(tb_duty[9]), .duty_b_3(tb_duty[10]), .duty_c_3(tb_duty[11]),
        .duty_a_4(tb_duty[12]), .duty_b_4(tb_duty[13]), .duty_c_4(tb_duty[14]),
        .duty_a_5(tb_duty[15]), .duty_b_5(tb_duty[16]), .duty_c_5(tb_duty[17]),
        .iq_act_0(tb_iq[0]), .iq_act_1(tb_iq[1]), .iq_act_2(tb_iq[2]),
        .iq_act_3(tb_iq[3]), .iq_act_4(tb_iq[4]), .iq_act_5(tb_iq[5]));

    initial begin clk=0; forever #5 clk=~clk; end

    // ---- udp_tx 契约接收模型（mbuf 扩到 256B，支持 op9 的 144B）----
    localparam M_IDLE=2'd0, M_WRITE=2'd1, M_DONE=2'd2;
    reg [1:0] mst; reg [10:0] mlen, mwidx; reg [7:0] mbuf [0:255]; reg pkt_sent;
    always @(*) m_udp_axis_ready = (mst==M_IDLE)||(mst==M_WRITE);
    always @(posedge clk) begin
        if (rst) begin mst<=M_IDLE; mwidx<=0; pkt_sent<=0; mlen<=0; end
        else begin
            pkt_sent<=0;
            case(mst)
            M_IDLE: if (m_udp_axis_valid) begin mlen<=cfg_udp_len; mbuf[0]<=m_udp_axis_data; mwidx<=1; mst<=M_WRITE; end
            M_WRITE: if (m_udp_axis_valid) begin mbuf[mwidx[7:0]]<=m_udp_axis_data; mwidx<=mwidx+1; if (m_udp_axis_last) mst<=M_DONE; end
            M_DONE: begin pkt_sent<=1; mwidx<=0; mst<=M_IDLE; end
            default: mst<=M_IDLE;
            endcase
        end
    end

    task drive_frame; input errlast; begin
        @(posedge clk);
        for (j=0;j<frn;j=j+1) begin
            s_udp_axis_data<=fr[j]; s_udp_axis_valid<=1'b1;
            s_udp_axis_last<=(j==frn-1); s_udp_axis_error<=(errlast&&(j==frn-1));
            @(posedge clk);
        end
        s_udp_axis_valid<=0; s_udp_axis_last<=0; s_udp_axis_error<=0;
    end endtask

    task await_pkt; output to; begin
        to=0; guard=0;
        while (pkt_sent!==1'b1) begin @(posedge clk); guard=guard+1; if (guard>4000) begin to=1; disable await_pkt; end end
    end endtask

    task expect_no_pkt; input integer cyc; input integer tid; reg seen; begin
        test_cnt=test_cnt+1; seen=0;
        for (guard=0;guard<cyc;guard=guard+1) begin @(posedge clk); if (pkt_sent===1'b1) seen=1; end
        if (!seen) begin $display("PASS t%0d: no packet as expected",tid); pass_cnt=pass_cnt+1; end
        else begin $display("FAIL t%0d: unexpected packet",tid); fail=1; end
    end endtask

    integer i;
    // 容差比较 (Q16.16). 位置/角度起步容差, 不够再放宽。
    localparam signed [31:0] TOL_POS = 32'sd512;   // ≈0.0078 m
    localparam signed [31:0] TOL_ANG = 32'sd1024;  // ≈0.0156 rad
    localparam signed [31:0] TOL_J = 32'sd1024;   // J 容差(定点+CORDIC); 不够再放宽
    reg signed [31:0] gold_j [0:35];
    function automatic within;
        input signed [31:0] got, exp, tol; reg signed [31:0] dlt;
        begin dlt = got-exp; if (dlt<0) dlt=-dlt; within = (dlt<=tol); end
    endfunction
    // 从 mbuf 第 n 个 4B(LE) 取 int32
    function signed [31:0] mword; input integer n;
        mword = {mbuf[4*n+3], mbuf[4*n+2], mbuf[4*n+1], mbuf[4*n]};
    endfunction

    // 校验 6 值: 位置用 TOL_POS, 角度用 TOL_ANG
    task fk_full_check;
        input signed [31:0] ex,ey,ez,erx,ery,erz; input integer tid; reg ok;
        begin
            test_cnt=test_cnt+1;
            build_write; drive_frame(1'b0); await_pkt(t_to);
            if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: ack to=%0d",tid,t_to); fail=1; end
            else begin
                build_read; drive_frame(1'b0); await_pkt(t_to);
                ok = (!t_to) && (mlen>=11'd24)
                   && within(mword(0),ex,TOL_POS) && within(mword(1),ey,TOL_POS) && within(mword(2),ez,TOL_POS)
                   && within(mword(3),erx,TOL_ANG) && within(mword(4),ery,TOL_ANG) && within(mword(5),erz,TOL_ANG);
                if (ok) begin $display("PASS t%0d: full pose ok",tid); pass_cnt=pass_cnt+1; end
                else begin $display("FAIL t%0d: got=(%0d,%0d,%0d,%0d,%0d,%0d) exp=(%0d,%0d,%0d,%0d,%0d,%0d)",
                    tid,mword(0),mword(1),mword(2),mword(3),mword(4),mword(5),ex,ey,ez,erx,ery,erz); fail=1; end
            end
        end
    endtask

    task fk_posonly_check;
        input signed [31:0] ex,ey,ez; input integer tid; reg ok;
        begin
            test_cnt=test_cnt+1;
            build_write; drive_frame(1'b0); await_pkt(t_to);
            if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: ack to=%0d",tid,t_to); fail=1; end
            else begin
                build_read; drive_frame(1'b0); await_pkt(t_to);
                ok = (!t_to) && (mlen>=11'd24)
                   && within(mword(0),ex,TOL_POS) && within(mword(1),ey,TOL_POS) && within(mword(2),ez,TOL_POS);
                if (ok) begin $display("PASS t%0d: pos-only ok",tid); pass_cnt=pass_cnt+1; end
                else begin $display("FAIL t%0d: pos got=(%0d,%0d,%0d) exp=(%0d,%0d,%0d)",
                    tid,mword(0),mword(1),mword(2),ex,ey,ez); fail=1; end
            end
        end
    endtask

    task build_write; begin   // 用 qv[0..5] 拼 op2 帧，并把 24B 负载存进 wpay
        fr[0]=OP_WRITE; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=qv[i][7:0];   fr[8+4*i]=qv[i][15:8];
            fr[9+4*i]=qv[i][23:16]; fr[10+4*i]=qv[i][31:24];
            wpay[4*i]=qv[i][7:0];   wpay[4*i+1]=qv[i][15:8];
            wpay[4*i+2]=qv[i][23:16]; wpay[4*i+3]=qv[i][31:24];
        end
        frn=31;
    end endtask

    task build_read; begin
        fr[0]=OP_READ; fr[1]=8'd8; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7;
    end endtask

    task build_readj; begin   // op9 读雅可比: [9][addr=0 LE4][len=144 LE2]
        fr[0]=8'd9; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd144; fr[6]=0; frn=7;
    end endtask

    reg [31:0] XDv [0:5], Kv [0:5]; // op10 写入的 x_des 和刚度矩阵对角
    reg [31:0] Dv [0:5], XVv [0:5]; // op10 96B 扩展: 阻尼矩阵对角 D 和笛卡尔速度 ẋ
    reg signed [31:0] gold_tau [0:5];
    localparam signed [31:0] TOL_TAU = 32'sd8192;  // 集成容差(覆盖 CORDIC 噪声), 非位精确; 单元 tb 才位精确

    // op12: torque_to_iq 复刻
    localparam signed [31:0] KT_INV_TB = 32'sd1310720;   // 同 torque_to_iq 默认
    reg signed [31:0] tau_rd [0:5];
    // 复刻 torque_to_iq: iq = sat16( (tau·KT_INV)>>16 , ±2000 )
    function signed [31:0] exp_iq; input signed [31:0] t; reg signed [63:0] p; reg signed [31:0] v;
        begin
            p = $signed(t) * KT_INV_TB;
            v = p[47:16];
            if (v > 32'sd2000)        exp_iq = 32'sd2000;
            else if (v < -32'sd2000)  exp_iq = -32'sd2000;
            else                      exp_iq = $signed({{16{v[15]}}, v[15:0]});
        end
    endfunction

    task build_imp_write_full; begin  // op10: [10][addr0 4B][len=96][x_des][K][D][ẋ]
        fr[0]=8'd10; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd96; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=XDv[i][7:0];   fr[8+4*i]=XDv[i][15:8];   fr[9+4*i]=XDv[i][23:16];  fr[10+4*i]=XDv[i][31:24];
            fr[31+4*i]=Kv[i][7:0];   fr[32+4*i]=Kv[i][15:8];   fr[33+4*i]=Kv[i][23:16];  fr[34+4*i]=Kv[i][31:24];
            fr[55+4*i]=Dv[i][7:0];   fr[56+4*i]=Dv[i][15:8];   fr[57+4*i]=Dv[i][23:16];  fr[58+4*i]=Dv[i][31:24];
            fr[79+4*i]=XVv[i][7:0];  fr[80+4*i]=XVv[i][15:8];  fr[81+4*i]=XVv[i][23:16]; fr[82+4*i]=XVv[i][31:24];
        end
        frn=103;
    end endtask

    task build_imp_read; begin     // op11: [11][addr=11 LE4][len=24 LE2]
        fr[0]=8'd11; fr[1]=8'd11; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7;
    end endtask

    task build_iq_read; begin     // op12: [12][addr 4B][len=24]
        fr[0]=8'd12; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7;
    end endtask

    task build_duty_read; begin   // op13: [13][addr 4B][len=36]
        fr[0]=8'd13; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd36; fr[6]=0; frn=7;
    end endtask

    task duty_check; input integer tid; reg ok; integer n; begin
        test_cnt=test_cnt+1;
        build_duty_read; drive_frame(1'b0); await_pkt(t_to);
        ok = (!t_to) && (mlen>=11'd36);
        for (n=0;n<18;n=n+1) begin
            if (mbuf[2*n]   !== tb_duty[n][7:0])         ok=0;   // 低字节
            if (mbuf[2*n+1] !== {6'd0, tb_duty[n][9:8]}) ok=0;   // 高字节
        end
        if (ok) begin $display("PASS t%0d: op13 duty readback ok",tid); pass_cnt=pass_cnt+1; end
        else begin
            $display("FAIL t%0d: op13 duty (to=%0d mlen=%0d)",tid,t_to,mlen);
            for (n=0;n<18;n=n+1) $display("   d[%0d] exp=%0d got lo=%0d hi=%0d",
                n, tb_duty[n], mbuf[2*n], mbuf[2*n+1]);
            fail=1;
        end
    end endtask

    // op2 写 q -> op8 读回 pose -> x_des=pose+delta -> op10 写 x_des+K(48B) -> op11 读 τ -> 容差比对
    task imp_check_stiffness; input integer tid; reg ok; integer n; begin
        test_cnt=test_cnt+1;
        build_write; drive_frame(1'b0); await_pkt(t_to);            // op2 -> ACK (锁存 J/pose)
        if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op2 ack",tid); fail=1; end
        else begin
            build_read; drive_frame(1'b0); await_pkt(t_to);         // op8 -> 读回 pose
            if (t_to || mlen<11'd24) begin $display("FAIL t%0d: op8 read",tid); fail=1; end
            else begin
                // x_des = 读回的 pose + delta_lsb
                XDv[0]=mword(0)+32'sd655; XDv[1]=mword(1)-32'sd655; XDv[2]=mword(2)+32'sd655;
                XDv[3]=mword(3)+32'sd0;   XDv[4]=mword(4)+32'sd1311; XDv[5]=mword(5)-32'sd655;
                for (n=0;n<6;n=n+1) begin Dv[n]=0; XVv[n]=0; end
                build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);  // op10 -> ACK
                if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op10 ack",tid); fail=1; end
                else begin
                    build_imp_read; drive_frame(1'b0); await_pkt(t_to);  // op11 -> 24B τ
                    ok = (!t_to) && (mlen>=11'd24);
                    for (n=0;n<6;n=n+1) if (!within(mword(n), gold_tau[n], TOL_TAU)) ok=0;
                    if (ok) begin $display("PASS t%0d: stiffness tau ok",tid); pass_cnt=pass_cnt+1; end
                    else begin $display("FAIL t%0d: tau got=(%0d,%0d,%0d,%0d,%0d,%0d) exp=(%0d,%0d,%0d,%0d,%0d,%0d)",
                        tid,mword(0),mword(1),mword(2),mword(3),mword(4),mword(5),
                        gold_tau[0],gold_tau[1],gold_tau[2],gold_tau[3],gold_tau[4],gold_tau[5]); fail=1; end
                end
            end
        end
    end endtask

    // op2 -> op8 读 pose -> x_des=pose+delta -> op10(96B,含 D/ẋ) -> op11 读 τ
    task imp_check_full; input integer tid; reg ok; integer n; begin
        test_cnt=test_cnt+1;
        build_write; drive_frame(1'b0); await_pkt(t_to);
        if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op2 ack",tid); fail=1; end
        else begin
            build_read; drive_frame(1'b0); await_pkt(t_to);
            if (t_to || mlen<11'd24) begin $display("FAIL t%0d: op8 read",tid); fail=1; end
            else begin
                XDv[0]=mword(0)+32'sd655; XDv[1]=mword(1)-32'sd655; XDv[2]=mword(2)+32'sd655;
                XDv[3]=mword(3)+32'sd0;   XDv[4]=mword(4)+32'sd1311; XDv[5]=mword(5)-32'sd655;
                build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);
                if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op10 ack",tid); fail=1; end
                else begin
                    build_imp_read; drive_frame(1'b0); await_pkt(t_to);
                    ok = (!t_to) && (mlen>=11'd24);
                    for (n=0;n<6;n=n+1) if (!within(mword(n), gold_tau[n], TOL_TAU)) ok=0;
                    if (ok) begin $display("PASS t%0d: full tau ok",tid); pass_cnt=pass_cnt+1; end
                    else begin $display("FAIL t%0d: tau got=(%0d,%0d,%0d,%0d,%0d,%0d) exp=(%0d,%0d,%0d,%0d,%0d,%0d)",
                        tid,mword(0),mword(1),mword(2),mword(3),mword(4),mword(5),
                        gold_tau[0],gold_tau[1],gold_tau[2],gold_tau[3],gold_tau[4],gold_tau[5]); fail=1; end
                end
            end
        end
    end endtask

    // op2->op8->x_des=pose+delta->op10->op11 读 τ->op12 读 iq->校验 iq==to_iq(τ_hw)
    task imp_check_iq; input integer tid; reg ok; integer n; begin
        test_cnt=test_cnt+1;
        build_write; drive_frame(1'b0); await_pkt(t_to);
        if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op2",tid); fail=1; end
        else begin
            build_read; drive_frame(1'b0); await_pkt(t_to);
            if (t_to||mlen<11'd24) begin $display("FAIL t%0d: op8",tid); fail=1; end
            else begin
                XDv[0]=mword(0)+32'sd1; XDv[1]=mword(1)-32'sd1; XDv[2]=mword(2)+32'sd1;
                XDv[3]=mword(3)+32'sd0; XDv[4]=mword(4)+32'sd2; XDv[5]=mword(5)-32'sd1;
                build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);   // op10(96B)
                if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op10",tid); fail=1; end
                else begin
                    build_imp_read; drive_frame(1'b0); await_pkt(t_to);     // op11 τ
                    for(n=0;n<6;n=n+1) tau_rd[n]=mword(n);
                    build_iq_read; drive_frame(1'b0); await_pkt(t_to);      // op12 iq
                    ok=(!t_to)&&(mlen>=11'd24);
                    for(n=0;n<6;n=n+1) if (mword(n) !== exp_iq(tau_rd[n])) ok=0;
                    if(ok) begin $display("PASS t%0d: iq=to_iq(tau) ok",tid); pass_cnt=pass_cnt+1; end
                    else begin $display("FAIL t%0d: iq != to_iq(tau)",tid);
                        for(n=0;n<6;n=n+1) $display("   iq[%0d] got=%0d exp=%0d (tau=%0d)",n,mword(n),exp_iq(tau_rd[n]),tau_rd[n]);
                        fail=1; end
                end
            end
        end
    end endtask

    // 写 q -> op9 读 144B -> 36 个 int32 对 gold_j[] 比对(容差)
    task fk_jac_check; input integer tid; reg ok; integer n; begin
        test_cnt=test_cnt+1;
        build_write; drive_frame(1'b0); await_pkt(t_to);          // op2 写 q -> ACK
        if (t_to || mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: jac write ack to=%0d",tid,t_to); fail=1; end
        else begin
            build_readj; drive_frame(1'b0); await_pkt(t_to);      // op9 读 -> 144B
            ok = (!t_to) && (mlen>=11'd144);
            for (n=0;n<36;n=n+1) if (!within(mword(n), gold_j[n], TOL_J)) ok=0;
            if (ok) begin $display("PASS t%0d: jacobian ok",tid); pass_cnt=pass_cnt+1; end
            else begin $display("FAIL t%0d: jac mismatch (e.g. n=1 got=%0d exp=%0d)",tid,mword(1),gold_j[1]); fail=1; end
        end
    end endtask

    // (M1 回环任务 write_then_echo 已删除；M2.x 用 fk_posonly_check / fk_full_check)

    task one_byte_acked; input [7:0] op; input integer tid; begin
        test_cnt=test_cnt+1;
        fr[0]=op; frn=1; drive_frame(1'b0); await_pkt(t_to);
        if (!t_to && mbuf[0]===ACK_BYTE) begin $display("PASS t%0d: ack op %0d (len=%0d)",tid,op,mlen); pass_cnt=pass_cnt+1; end
        else begin $display("FAIL t%0d: op %0d to=%0d mbuf0=0x%02x",tid,op,t_to,mbuf[0]); fail=1; end
    end endtask

    // ============ 自适应阻抗 op15/16/17 ============
    task build_op16_prior; input [31:0] kenv; input [31:0] pcov; begin   // [16][addr4][len48][Kprior×6][Pprior×6]
        fr[0]=8'd16; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd48; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=kenv[7:0];  fr[8+4*i]=kenv[15:8];  fr[9+4*i]=kenv[23:16];  fr[10+4*i]=kenv[31:24];
            fr[31+4*i]=pcov[7:0]; fr[32+4*i]=pcov[15:8]; fr[33+4*i]=pcov[23:16]; fr[34+4*i]=pcov[31:24];
        end
        frn=55;
    end endtask

    task build_op15; input en; input [31:0] qe; input [31:0] kn; input [31:0] dn; begin // [15][addr4][len73][en1][q_err×6][Ks_nom×6][Ds_nom×6]
        fr[0]=8'd15; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd73; fr[6]=0;
        fr[7]= en ? 8'd1 : 8'd0;
        for (i=0;i<6;i=i+1) begin
            fr[8+4*i]=qe[7:0];  fr[9+4*i]=qe[15:8];  fr[10+4*i]=qe[23:16]; fr[11+4*i]=qe[31:24];
            fr[32+4*i]=kn[7:0]; fr[33+4*i]=kn[15:8]; fr[34+4*i]=kn[23:16]; fr[35+4*i]=kn[31:24];
            fr[56+4*i]=dn[7:0]; fr[57+4*i]=dn[15:8]; fr[58+4*i]=dn[23:16]; fr[59+4*i]=dn[31:24];
        end
        frn=80;
    end endtask

    task build_op17_read; begin   // [17][addr4][len72]
        fr[0]=8'd17; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd72; fr[6]=0; frn=7;
    end endtask

    // T51: op16 注入 K_env 先验 -> op17 读回 (K_env_est 应=先验)
    task prior_readback_check; input integer tid; reg ok; begin
        test_cnt=test_cnt+1;
        build_op16_prior(32'sd131072000, 32'sd65536000); drive_frame(1'b0); await_pkt(t_to); // K_env=2000,P=1000
        if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op16 ack to=%0d",tid,t_to); fail=1; end
        else begin
            build_op17_read; drive_frame(1'b0); await_pkt(t_to);
            ok = (!t_to) && (mlen>=11'd72) && (mword(0)===32'sd131072000) && (mword(5)===32'sd131072000);
            if (ok) begin $display("PASS t%0d: prior readback K_env=%0d (2000.0)",tid,mword(0)); pass_cnt=pass_cnt+1; end
            else begin $display("FAIL t%0d: prior readback K_env0=%0d K_env5=%0d (exp 131072000) to=%0d mlen=%0d",tid,mword(0),mword(5),t_to,mlen); fail=1; end
        end
    end endtask

    // T50: op2->op16(先验)->op10(设 τ_cmd)->op15(enable,片上算 τ_ext,跑自适应)->op17 读
    //   核对反比律关系 Ks_out·K_env ≈ Ks_nom² 与 Ds_out/Ks_out ≈ Ds_nom/Ks_nom (内部定义,免外部黄金值)
    task adapt_check; input integer tid; reg ok; integer n; real rks, rkenv, rds, rel1, rel2; begin
        test_cnt=test_cnt+1;
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        build_write; drive_frame(1'b0); await_pkt(t_to);                              // op2 -> 锁存 J/pose
        build_read; drive_frame(1'b0); await_pkt(t_to);                                // op8 -> 读回 pose
        XDv[0]=mword(0)+32'sd655; XDv[1]=mword(1)-32'sd655; XDv[2]=mword(2)+32'sd655;  // x_des=pose+小 delta (Δx 小 -> τ_cmd 合理)
        XDv[3]=mword(3);          XDv[4]=mword(4)+32'sd1311; XDv[5]=mword(5)-32'sd655;
        for (n=0;n<6;n=n+1) begin Kv[n]=32'sd6553600; Dv[n]=0; XVv[n]=0; end
        build_op16_prior(32'sd78643200, 32'sd65536000); drive_frame(1'b0); await_pkt(t_to); // op16 K_env=1200,P=1000
        build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);                      // op10 -> tau_reg(τ_cmd)
        for (n=0;n<6;n=n+1) tb_iq[n]=16'sd100;                                          // iq_actual 制造接触
        build_op15(1'b1, 32'sd3277, 32'sd6553600, 32'sd655360); drive_frame(1'b0); await_pkt(t_to); // op15 enable,q_err=0.05,Ks_nom=100,Ds_nom=10
        if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op15 ack to=%0d",tid,t_to); fail=1; end
        else begin
            build_op17_read; drive_frame(1'b0); await_pkt(t_to);                        // op17 -> 72B
            // mword 0..5=K_env_est, 6..11=Ks_out, 12..17=Ds_out
            rkenv=$itor(mword(0))/65536.0; rks=$itor(mword(6))/65536.0; rds=$itor(mword(12))/65536.0;
            ok = (!t_to) && (mlen>=11'd72);
            if (mword(0)===32'sd78643200) ok=0;          // K_env 仍=1200 => 没更新
            if (mword(0) <= 0) ok=0;                       // K_env 必须 >0
            rel1=(rks*rkenv-10000.0)/10000.0; if(rel1<0) rel1=-rel1;       // Ks_out·K_env ~ Ks_nom²
            if (rel1>0.05) ok=0;
            rel2=(rds/rks-0.1)/0.1; if(rel2<0) rel2=-rel2;                  // Ds_out/Ks_out ~ Ds_nom/Ks_nom=0.1
            if (rel2>0.05) ok=0;
            if (ok) begin $display("PASS t%0d: adapt K_env=%.2f Ks_out=%.3f Ds_out=%.3f rel(Ks*Kenv~10000)=%.4f",tid,rkenv,rks,rds,rel1); pass_cnt=pass_cnt+1; end
            else begin $display("FAIL t%0d: adapt K_env=%.2f Ks_out=%.3f Ds_out=%.3f rel1=%.4f rel2=%.4f to=%0d mlen=%0d",tid,rkenv,rks,rds,rel1,rel2,t_to,mlen); fail=1; end
        end
    end endtask

    // T52: 闭环 mux 验证. adaptive_en=1 后, op10 的 payload K 应被忽略(用 Ks_out), 两次不同 K 的 τ 必相同.
    task mux_diff_check; input integer tid; reg ok; integer n; reg signed [31:0] tauA [0:5]; begin
        test_cnt=test_cnt+1;
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        build_write; drive_frame(1'b0); await_pkt(t_to);
        build_op16_prior(32'sd78643200, 32'sd65536000); drive_frame(1'b0); await_pkt(t_to);
        XDv[0]=32'sd655; XDv[1]=0; XDv[2]=32'sd655; XDv[3]=0; XDv[4]=32'sd1311; XDv[5]=0;
        for (n=0;n<6;n=n+1) begin Kv[n]=32'sd6553600; Dv[n]=0; XVv[n]=0; end
        build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);
        for (n=0;n<6;n=n+1) tb_iq[n]=16'sd100;
        build_op15(1'b1, 32'sd3277, 32'sd6553600, 32'sd655360); drive_frame(1'b0); await_pkt(t_to); // enable=1, 得 Ks_out
        // op10 K=A=100 -> op11 τ_A
        for (n=0;n<6;n=n+1) Kv[n]=32'sd6553600;
        build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);
        build_imp_read; drive_frame(1'b0); await_pkt(t_to);
        for (n=0;n<6;n=n+1) tauA[n]=mword(n);
        // op10 K=B≈999 (与 A 天差地别) -> op11 τ_B ; mux 忽略 payload K => τ_B 应===τ_A
        for (n=0;n<6;n=n+1) Kv[n]=32'sd65470464;
        build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);
        build_imp_read; drive_frame(1'b0); await_pkt(t_to);
        ok=(!t_to)&&(mlen>=11'd24);
        for (n=0;n<6;n=n+1) if (mword(n)!==tauA[n]) ok=0;
        if (ok) begin $display("PASS t%0d: closed-loop mux 忽略 payload K (tau_A===tau_B, 用 Ks_out)",tid); pass_cnt=pass_cnt+1; end
        else begin $display("FAIL t%0d: mux: payload K 改变了 τ (adaptive_en=1 时不应)",tid);
            for (n=0;n<6;n=n+1) $display("   tau[%0d] A=%0d B=%0d",n,tauA[n],mword(n)); fail=1; end
    end endtask

    // ============ E2 片上周期计数器 op18 ============
    task build_op18_read; begin   // [18][addr4][len=24]
        fr[0]=8'd18; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7;
    end endtask

    // T60/T61: op18 读 runs 基线 -> 完整前向链 (op2 -> op10) -> op18 复读:
    //   last/min/max 必须 ===549 (论文 Table III: 4 拍 FSM 锁存 + fk422 + imp119 + tiq4),
    //   runs 恰好 +1, 魔数 E2E2E2E2, 尾字 0。
    task e2_cycle_check; input integer tid; reg ok; reg signed [31:0] runs0; begin
        test_cnt=test_cnt+1;
        build_op18_read; drive_frame(1'b0); await_pkt(t_to);          // 基线读
        if (t_to || mlen<11'd24 || mword(4)!==32'shE2E2E2E2) begin
            $display("FAIL t%0d: op18 baseline to=%0d magic=%08x",tid,t_to,mword(4)); fail=1;
        end else begin
            runs0 = mword(3);
            qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
            build_write; drive_frame(1'b0); await_pkt(t_to);          // op2 -> FK (窗口开)
            if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op2 ack",tid); fail=1; end
            else begin
                XDv[0]=32'sd655; XDv[1]=0; XDv[2]=32'sd655; XDv[3]=0; XDv[4]=32'sd1311; XDv[5]=0;
                for (i=0;i<6;i=i+1) begin Kv[i]=32'sd6553600; Dv[i]=32'sd655360; XVv[i]=0; end
                build_imp_write_full; drive_frame(1'b0); await_pkt(t_to);   // op10 -> imp+tiq (窗口合)
                if (t_to||mbuf[0]!==ACK_BYTE) begin $display("FAIL t%0d: op10 ack",tid); fail=1; end
                else begin
                    build_op18_read; drive_frame(1'b0); await_pkt(t_to);    // 复读统计
                    ok = (!t_to) && (mlen>=11'd24)
                       && (mword(0)===32'sd549)                 // last
                       && (mword(1)===32'sd549)                 // min (全部历史闭合均须 549)
                       && (mword(2)===32'sd549)                 // max
                       && (mword(3)===runs0+32'sd1)             // runs 恰 +1
                       && (mword(4)===32'shE2E2E2E2)            // 魔数
                       && (mword(5)===32'sd0);                  // 尾字
                    if (ok) begin $display("PASS t%0d: op18 E2 cycles last=%0d min=%0d max=%0d runs=%0d->%0d magic=%08x",
                        tid,mword(0),mword(1),mword(2),runs0,mword(3),mword(4)); pass_cnt=pass_cnt+1; end
                    else begin $display("FAIL t%0d: op18 last=%0d min=%0d max=%0d runs=%0d(base %0d) magic=%08x tail=%0d (exp 549/549/549/+1/E2E2E2E2/0)",
                        tid,mword(0),mword(1),mword(2),mword(3),runs0,mword(4),mword(5)); fail=1; end
                end
            end
        end
    end endtask

    initial begin
        test_cnt=0; pass_cnt=0; fail=0;
        s_udp_axis_data=0; s_udp_axis_valid=0; s_udp_axis_last=0; s_udp_axis_error=0;
        for (i=0;i<18;i=i+1) tb_duty[i]=10'd0;  // duty 初始值全 0
        for (i=0;i<6;i=i+1) tb_iq[i]=16'sd0;     // iq_actual 初始全 0
        rst=1; repeat(12) @(posedge clk); rst=0; repeat(5) @(posedge clk);

        // home q=0 -> 只查位置（rx≈±π 边界不稳，不查欧拉角）
        qv[0]=0; qv[1]=0; qv[2]=0; qv[3]=0; qv[4]=0; qv[5]=0;
        fk_posonly_check(32'sd20381, 32'sd0, -32'sd5308, 1);
        // nontrivial -> 查完整位姿（位置+欧拉角）。rx≈0（此姿态 roll 近 0，-1 为 Q16.16 量化）
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        fk_full_check(32'sd20053, 32'sd11578, -32'sd8661, -32'sd1, -32'sd85786, -32'sd171571, 2);
        // generic -> 第二组完整位姿（不同 ry=-0.28rad，验证 ry 增益补偿全程成立，非仅对单点）
        qv[0]=32'sd22877; qv[1]=32'sd11439; qv[2]=-32'sd34315; qv[3]=32'sd17157; qv[4]=32'sd45751; qv[5]=-32'sd28598;
        fk_full_check(32'sd16006, 32'sd5188, -32'sd3284, 32'sd190132, -32'sd18642, 32'sd38635, 6);

        // op9 真实雅可比: nontrivial
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        gold_j[0]=-32'sd11578; gold_j[1]=32'sd16922; gold_j[2]=32'sd8895; gold_j[3]=32'sd1802; gold_j[4]=-32'sd808; gold_j[5]=0;
        gold_j[6]=32'sd20053; gold_j[7]=32'sd9770; gold_j[8]=32'sd5136; gold_j[9]=-32'sd3121; gold_j[10]=-32'sd466; gold_j[11]=0;
        gold_j[12]=0; gold_j[13]=32'sd19551; gold_j[14]=32'sd10283; gold_j[15]=0; gold_j[16]=32'sd3481; gold_j[17]=0;
        gold_j[18]=0; gold_j[19]=32'sd32768; gold_j[20]=32'sd32768; gold_j[21]=32'sd14689; gold_j[22]=32'sd32768; gold_j[23]=32'sd54822;
        gold_j[24]=0; gold_j[25]=-32'sd56756; gold_j[26]=-32'sd56756; gold_j[27]=32'sd8481; gold_j[28]=-32'sd56756; gold_j[29]=32'sd31652;
        gold_j[30]=32'sd65536; gold_j[31]=0; gold_j[32]=0; gold_j[33]=-32'sd63303; gold_j[34]=0; gold_j[35]=32'sd16962;
        fk_jac_check(7);
        // op9 真实雅可比: generic
        qv[0]=32'sd22877; qv[1]=32'sd11439; qv[2]=-32'sd34315; qv[3]=32'sd17157; qv[4]=32'sd45751; qv[5]=-32'sd28598;
        gold_j[0]=-32'sd5188; gold_j[1]=32'sd13309; gold_j[2]=32'sd15448; gold_j[3]=32'sd236; gold_j[4]=32'sd3344; gold_j[5]=0;
        gold_j[6]=32'sd16006; gold_j[7]=32'sd4844; gold_j[8]=32'sd5623; gold_j[9]=-32'sd2295; gold_j[10]=32'sd457; gold_j[11]=0;
        gold_j[12]=0; gold_j[13]=32'sd13211; gold_j[14]=32'sd304; gold_j[15]=32'sd205; gold_j[16]=32'sd1265; gold_j[17]=0;
        gold_j[18]=0; gold_j[19]=32'sd22415; gold_j[20]=32'sd22415; gold_j[21]=-32'sd21063; gold_j[22]=32'sd6674; gold_j[23]=32'sd23523;
        gold_j[24]=0; gold_j[25]=-32'sd61583; gold_j[26]=-32'sd61583; gold_j[27]=-32'sd7666; gold_j[28]=-32'sd64937; gold_j[29]=-32'sd3040;
        gold_j[30]=32'sd65536; gold_j[31]=0; gold_j[32]=0; gold_j[33]=-32'sd61584; gold_j[34]=32'sd5801; gold_j[35]=-32'sd61093;
        fk_jac_check(8);
        // op10/op11 刚度: nontrivial, K=[100,100,100,10,10,10], x_des=pose+delta
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        Kv[0]=32'sd6553600; Kv[1]=32'sd6553600; Kv[2]=32'sd6553600; Kv[3]=32'sd655360; Kv[4]=32'sd655360; Kv[5]=32'sd655360;
        gold_tau[0]=-32'sd38164; gold_tau[1]=32'sd15333; gold_tau[2]=32'sd2679;
        gold_tau[3]=32'sd12942; gold_tau[4]=-32'sd8218; gold_tau[5]=32'sd4635;
        imp_check_stiffness(20);
        // op10/op11 阻尼: 同 q/K, 加 D=[10,10,10,1,1,1], ẋ
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        Kv[0]=32'sd6553600; Kv[1]=32'sd6553600; Kv[2]=32'sd6553600; Kv[3]=32'sd655360; Kv[4]=32'sd655360; Kv[5]=32'sd655360;
        Dv[0]=32'sd655360; Dv[1]=32'sd655360; Dv[2]=32'sd655360; Dv[3]=32'sd65536; Dv[4]=32'sd65536; Dv[5]=32'sd65536;
        XVv[0]=32'sd655; XVv[1]=-32'sd655; XVv[2]=32'sd328; XVv[3]=0; XVv[4]=-32'sd655; XVv[5]=32'sd328;
        gold_tau[0]=-32'sd35331; gold_tau[1]=32'sd13072; gold_tau[2]=32'sd1222;
        gold_tau[3]=32'sd12851; gold_tau[4]=-32'sd8925; gold_tau[5]=32'sd4867;
        imp_check_full(21);
        // op12: 小 delta 使 iq 不饱和; 校验 iq==to_iq(τ_hw)
        qv[0]=32'sd34315; qv[1]=-32'sd51472; qv[2]=32'sd68629; qv[3]=0; qv[4]=32'sd102944; qv[5]=0;
        Kv[0]=32'sd6553600; Kv[1]=32'sd6553600; Kv[2]=32'sd6553600; Kv[3]=32'sd655360; Kv[4]=32'sd655360; Kv[5]=32'sd655360;
        Dv[0]=32'sd655360; Dv[1]=32'sd655360; Dv[2]=32'sd655360; Dv[3]=32'sd65536; Dv[4]=32'sd65536; Dv[5]=32'sd65536;
        XVv[0]=32'sd6; XVv[1]=-32'sd6; XVv[2]=32'sd3; XVv[3]=0; XVv[4]=-32'sd6; XVv[5]=32'sd3;
        imp_check_iq(30);

        // op13: 驱动 18 个不同占空比, 读回校验打包
        for (i=0;i<18;i=i+1) tb_duty[i] = 10'd100 + i * 10'd23;  // 100,123,146,...,491 各不同, <1024
        @(posedge clk); @(posedge clk);                            // 等 duty_reg 快照生效
        duty_check(40);

        one_byte_acked(OP_START,3);
        fr[0]=OP_NOP7; frn=1; drive_frame(1'b0); expect_no_pkt(400,4);
        qv[0]=32'd1; qv[1]=32'd2; qv[2]=32'd3; qv[3]=32'd4; qv[4]=32'd5; qv[5]=32'd6;
        build_write; drive_frame(1'b1); expect_no_pkt(400,5);

        // ==== 自适应阻抗集成 op15/16/17 (放最后: adaptive_en 会被置 1) ====
        prior_readback_check(50);   // op16 先验 -> op17 读回 K_env
        adapt_check(51);            // op15 片上 τ_ext + RLS 自适应 -> 反比律关系核对
        mux_diff_check(52);         // 闭环 mux: adaptive_en=1 时 cart_impedance 用 Ks_out(忽略 op10 payload K)

        // ==== E2 片上周期计数器 op18 (T60/T61: 连续两窗均须 549, runs 逐次+1;
        //      此前 T20/21/30/51/52 已闭合过多个窗口, min==max==549 同时锁定历史全部窗口) ====
        e2_cycle_check(60);
        e2_cycle_check(61);

        $display("==== %0d/%0d PASS ====", pass_cnt, test_cnt);
        if (!fail && pass_cnt==test_cnt) $display("ALL TESTS PASSED");
        else                             $display("SOME TESTS FAILED");
        $finish;
    end
endmodule
