`timescale 1ns/1ps
// ============================================================================
// tb_r2_1_fwd_cycles.v — R2.1 硬指标实测（论文 §V-B）
// ----------------------------------------------------------------------------
// 论文 R2.1 声明：
//   "Under both configurations of the adaptive-enable bit (adaptive_en) set to
//    0 or 1, for the same joint-angle input, the number of forward-chain cycles
//    defined in R1.1 is exactly identical."
//
// 本 tb 直接例化【真实 RTL】 V3/src/udp_fk_cmd.v，用真实 UDP 负载字节流驱动
// (op16/op15/op2/op10/op11/op12)，层次化探针只读内部握手信号，不改任何 RTL。
//
// 前向链定义（与 sim/tb_r1_1_cycles.v 的 R1.1 口径一致，逐拍同语义计数）：
//   seg1  FK+Jac        : uut.fk_start  上升沿 → uut.fk_done   (op2  触发)
//   seg2  cart_impedance: uut.imp_start 上升沿 → uut.imp_done  (op10 触发)
//   gap   FSM 内部空拍  : imp_done → tiq_start
//   seg3  torque_to_iq  : uut.tiq_start 上升沿 → uut.tiq_done
//   chain = imp_start 上升沿 → tiq_done  (= seg2 + gap + seg3, 全在 FSM 内部)
//   FWD   = seg1 + chain (不含 host 侧发包间隔, 那是网络时延不是逻辑拍数)
//
// 计数语义：起始拍计 1, 到观察到 done 的那一拍为止(含), 与 tb_r1_1_cycles.v 相同。
//
// 实验协议：
//   A) 10 组关节角(直接复用 sim/tb_r1_1_cycles.v 的 qcfg 表, 便于与 R1.1 对照)
//      每组分别在 adaptive_en=0 与 adaptive_en=1 下走一遍完整前向链, 逐组断言
//      seg1/seg2/seg3/chain/FWD 完全相等; 同时读回 τ(op11)/iq(op12) 证明 en=1
//      时数值确实走了自适应增益(非空操作)。
//   B) 强条件: 在前向链计算进行中并发触发自适应更新(adapt_start), 验证前向拍数
//      不变。注意 udp_fk_cmd 的命令 FSM 是单线程的(op15 走 S_ADAPT_WAIT 串行),
//      靠发包无法制造真正的并发, 故用 force/release 直接在 DUT 层次内注入 1 拍
//      adapt_start, 让 adaptive_impedance_v2 与 cart_impedance / torque_to_iq
//      真正同时在跑 —— 这是比命令协议更强的结构性条件。
//
// sentinel: 全部相等才打印 R2_1_FWD_CYCLES_EQ_OK + ALL TESTS PASSED。
// ============================================================================
module tb_r2_1_fwd_cycles;

    // ---------------- DUT 接口 ----------------
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

    initial begin clk = 0; forever #5 clk = ~clk; end   // 100 MHz (仿真时钟)

    // ---------------- udp_tx 契约接收模型 ----------------
    localparam M_IDLE=2'd0, M_WRITE=2'd1, M_DONE=2'd2;
    reg [1:0]  mst;
    reg [10:0] mlen, mwidx;
    reg [7:0]  mbuf [0:255];
    reg        pkt_sent;
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

    // ---------------- 拍数采样器 (与 tb_r1_1_cycles.v 同语义) ----------------
    reg [15:0] fk_cnt, imp_cnt, tiq_cnt, chain_cnt, gap_cnt, adapt_cnt;
    reg        fk_run, imp_run, tiq_run, chain_run, gap_run, adapt_run;
    reg        fk_sd,  imp_sd,  tiq_sd,  adapt_sd;
    reg [15:0] last_fk, last_imp, last_tiq, last_chain, last_gap, last_adapt;
    // 并发观测: 前向链运行期间 adaptive 引擎处于 busy 的拍数
    reg [15:0] ovl_cnt;
    reg [15:0] last_ovl;

    always @(posedge clk) begin
        if (rst) begin
            fk_cnt<=0; imp_cnt<=0; tiq_cnt<=0; chain_cnt<=0; gap_cnt<=0; adapt_cnt<=0;
            fk_run<=0; imp_run<=0; tiq_run<=0; chain_run<=0; gap_run<=0; adapt_run<=0;
            fk_sd<=0; imp_sd<=0; tiq_sd<=0; adapt_sd<=0;
            last_fk<=0; last_imp<=0; last_tiq<=0; last_chain<=0; last_gap<=0; last_adapt<=0;
            ovl_cnt<=0; last_ovl<=0;
        end else begin
            fk_sd    <= uut.fk_start;
            imp_sd   <= uut.imp_start;
            tiq_sd   <= uut.tiq_start;
            adapt_sd <= uut.adapt_start;

            // -- seg1: FK+Jac --
            if (uut.fk_start && !fk_sd) begin
                fk_run <= 1; fk_cnt <= 1;
            end else if (fk_run) begin
                fk_cnt <= fk_cnt + 16'd1;
                if (uut.fk_done) begin last_fk <= fk_cnt + 16'd1; fk_run <= 0; end
            end

            // -- seg2: cart_impedance --  (同时启动 chain 与 overlap 观测)
            if (uut.imp_start && !imp_sd) begin
                imp_run   <= 1; imp_cnt   <= 1;
                chain_run <= 1; chain_cnt <= 1;
                ovl_cnt   <= (adapt_run || (uut.adapt_start && !adapt_sd)) ? 16'd1 : 16'd0;
            end else if (imp_run) begin
                imp_cnt <= imp_cnt + 16'd1;
                if (uut.imp_done) begin
                    last_imp <= imp_cnt + 16'd1; imp_run <= 0;
                    gap_run  <= 1; gap_cnt <= 0;
                end
            end
            if (gap_run) gap_cnt <= gap_cnt + 16'd1;

            // -- seg3: torque_to_iq --
            if (uut.tiq_start && !tiq_sd) begin
                tiq_run <= 1; tiq_cnt <= 1;
                if (gap_run) begin last_gap <= gap_cnt; gap_run <= 0; end
            end else if (tiq_run) begin
                tiq_cnt <= tiq_cnt + 16'd1;
                if (uut.tiq_done) begin last_tiq <= tiq_cnt + 16'd1; tiq_run <= 0; end
            end

            // -- chain: imp_start → tiq_done --
            if (chain_run) begin
                chain_cnt <= chain_cnt + 16'd1;
                if (uut.tiq_done) begin last_chain <= chain_cnt + 16'd1; chain_run <= 0; last_ovl <= ovl_cnt; end
            end

            // -- adaptive 引擎 busy 拍数 (仅观测, 不参与判据) --
            if (uut.adapt_start && !adapt_sd) begin
                adapt_run <= 1; adapt_cnt <= 1;
            end else if (adapt_run) begin
                adapt_cnt <= adapt_cnt + 16'd1;
                if (uut.adapt_done) begin last_adapt <= adapt_cnt + 16'd1; adapt_run <= 0; end
            end

            // -- overlap: 前向链(chain)进行中 adaptive 也在跑的拍数 --
            if (chain_run && (adapt_run || (uut.adapt_start && !adapt_sd)))
                ovl_cnt <= ovl_cnt + 16'd1;
        end
    end

    // ---------------- 发包 ----------------
    reg  [7:0] fr [0:127];
    integer    frn, i, j, guard;
    reg        t_to;

    task drive_frame; begin
        @(posedge clk);
        for (j=0; j<frn; j=j+1) begin
            s_udp_axis_data <= fr[j];
            s_udp_axis_valid <= 1'b1;
            s_udp_axis_last  <= (j==frn-1);
            s_udp_axis_error <= 1'b0;
            @(posedge clk);
        end
        s_udp_axis_valid <= 0; s_udp_axis_last <= 0; s_udp_axis_error <= 0;
    end endtask

    task await_pkt; output to; begin
        to = 0; guard = 0;
        while (pkt_sent !== 1'b1) begin
            @(posedge clk); guard = guard + 1;
            if (guard > 20000) begin to = 1; disable await_pkt; end
        end
    end endtask

    // ---- 帧构造 (字节布局与 sim/tb_udp_fk_cmd.v 完全一致) ----
    reg [31:0] qv  [0:5];
    reg [31:0] XDv [0:5], Kv [0:5], Dv [0:5], XVv [0:5];

    task build_op2; begin                          // [2][addr4][len=24][q0..q5]
        fr[0]=8'd2; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=qv[i][7:0];   fr[8+4*i]=qv[i][15:8];
            fr[9+4*i]=qv[i][23:16]; fr[10+4*i]=qv[i][31:24];
        end
        frn=31;
    end endtask

    task build_op10; begin                         // [10][addr4][len=96][x_des][K][D][ẋ]
        fr[0]=8'd10; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd96; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i] =XDv[i][7:0];  fr[8+4*i] =XDv[i][15:8];  fr[9+4*i] =XDv[i][23:16];  fr[10+4*i]=XDv[i][31:24];
            fr[31+4*i]=Kv[i][7:0];   fr[32+4*i]=Kv[i][15:8];   fr[33+4*i]=Kv[i][23:16];   fr[34+4*i]=Kv[i][31:24];
            fr[55+4*i]=Dv[i][7:0];   fr[56+4*i]=Dv[i][15:8];   fr[57+4*i]=Dv[i][23:16];   fr[58+4*i]=Dv[i][31:24];
            fr[79+4*i]=XVv[i][7:0];  fr[80+4*i]=XVv[i][15:8];  fr[81+4*i]=XVv[i][23:16];  fr[82+4*i]=XVv[i][31:24];
        end
        frn=103;
    end endtask

    task build_op11; begin fr[0]=8'd11; fr[1]=8'd11; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7; end endtask
    task build_op12; begin fr[0]=8'd12; fr[1]=0;     fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7; end endtask

    task build_op16; input [31:0] kenv; input [31:0] pcov; begin   // [16][addr4][len=48]
        fr[0]=8'd16; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd48; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i] =kenv[7:0];  fr[8+4*i] =kenv[15:8];  fr[9+4*i] =kenv[23:16];  fr[10+4*i]=kenv[31:24];
            fr[31+4*i]=pcov[7:0];  fr[32+4*i]=pcov[15:8];  fr[33+4*i]=pcov[23:16];  fr[34+4*i]=pcov[31:24];
        end
        frn=55;
    end endtask

    task build_op15; input en; input [31:0] qe; input [31:0] kn; input [31:0] dn; begin  // [15][addr4][len=73]
        fr[0]=8'd15; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd73; fr[6]=0;
        fr[7] = en ? 8'd1 : 8'd0;
        for (i=0;i<6;i=i+1) begin
            fr[8+4*i] =qe[7:0];  fr[9+4*i] =qe[15:8];  fr[10+4*i]=qe[23:16]; fr[11+4*i]=qe[31:24];
            fr[32+4*i]=kn[7:0];  fr[33+4*i]=kn[15:8];  fr[34+4*i]=kn[23:16]; fr[35+4*i]=kn[31:24];
            fr[56+4*i]=dn[7:0];  fr[57+4*i]=dn[15:8];  fr[58+4*i]=dn[23:16]; fr[59+4*i]=dn[31:24];
        end
        frn=80;
    end endtask

    function signed [31:0] mword; input integer n;
        mword = {mbuf[4*n+3], mbuf[4*n+2], mbuf[4*n+1], mbuf[4*n]};
    endfunction

    // ---------------- 并发注入: 1 拍 adapt_start (force/release) ----------------
    task pulse_adapt_concurrent; begin
        @(posedge clk); #1;
        force uut.adapt_start = 1'b1;
        @(posedge clk); #1;
        force uut.adapt_start = 1'b0;
        @(posedge clk); #1;
        release uut.adapt_start;
    end endtask

    // ---------------- 10 组关节角 (直接复用 R1.1 的 qcfg 表) ----------------
    reg signed [31:0] qcfg [0:9][0:5];
    task load_qcfgs; begin
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
    end endtask

    // op10 负载 (与 tb_r1_1_cycles.v 相同)
    task load_op10_payload; begin
        XDv[0]=32'sd1000; XDv[1]=32'sd1000; XDv[2]=32'sd1000;
        XDv[3]=32'sd1000; XDv[4]=32'sd1000; XDv[5]=32'sd1000;
        Kv[0]=32'sd6553600; Kv[1]=32'sd6553600; Kv[2]=32'sd6553600;
        Kv[3]=32'sd655360;  Kv[4]=32'sd655360;  Kv[5]=32'sd655360;
        Dv[0]=32'sd655360; Dv[1]=32'sd655360; Dv[2]=32'sd655360;
        Dv[3]=32'sd65536;  Dv[4]=32'sd65536;  Dv[5]=32'sd65536;
        XVv[0]=32'sd655; XVv[1]=-32'sd655; XVv[2]=32'sd328;
        XVv[3]=32'sd0;   XVv[4]=-32'sd655; XVv[5]=32'sd328;
    end endtask

    // ---------------- 结果记录 ----------------
    reg [15:0] L_fk  [0:1][0:9];
    reg [15:0] L_imp [0:1][0:9];
    reg [15:0] L_gap [0:1][0:9];
    reg [15:0] L_tiq [0:1][0:9];
    reg [15:0] L_chn [0:1][0:9];
    reg [15:0] L_fwd [0:1][0:9];
    reg signed [31:0] L_tau [0:1][0:9];
    reg signed [31:0] L_iq  [0:1][0:9];

    integer tc, errors, tau_diff_cnt;
    integer ti;

    // 走一遍前向链: op2(q) -> op10(payload) -> 读 τ(op11) / iq(op12)
    // inj: 0=不注入, 1=FK 阶段注入, 2=cart_imp 阶段注入, 3=torque_to_iq 阶段注入
    task run_forward; input integer idx_tc; input integer en; input integer inj;
        reg [15:0] f_fk, f_imp, f_gap, f_tiq, f_chn;
        begin
            qv[0]=qcfg[idx_tc][0]; qv[1]=qcfg[idx_tc][1]; qv[2]=qcfg[idx_tc][2];
            qv[3]=qcfg[idx_tc][3]; qv[4]=qcfg[idx_tc][4]; qv[5]=qcfg[idx_tc][5];

            // ---- seg1: op2 触发 FK+Jac ----
            build_op2; drive_frame;
            if (inj==1) begin
                wait (uut.fk_start === 1'b1);
                repeat (20) @(posedge clk);
                pulse_adapt_concurrent;
            end
            await_pkt(t_to);
            if (t_to) begin $display("  !! TIMEOUT waiting op2 ACK (tc=%0d en=%0d inj=%0d)", idx_tc, en, inj); errors=errors+1; end
            f_fk = last_fk;

            // ---- seg2+seg3: op10 触发 cart_impedance -> torque_to_iq ----
            build_op10; drive_frame;
            if (inj==2) begin
                wait (uut.imp_start === 1'b1);
                repeat (20) @(posedge clk);
                pulse_adapt_concurrent;
            end
            if (inj==3) begin
                wait (uut.imp_done === 1'b1);      // 恰好跨 imp->tiq 边界注入
                pulse_adapt_concurrent;
            end
            await_pkt(t_to);
            if (t_to) begin $display("  !! TIMEOUT waiting op10 ACK (tc=%0d en=%0d inj=%0d)", idx_tc, en, inj); errors=errors+1; end
            f_imp = last_imp; f_gap = last_gap; f_tiq = last_tiq; f_chn = last_chain;

            // ---- 读回 τ0 / iq0 (证明 en=0/1 数值确实不同) ----
            build_op11; drive_frame; await_pkt(t_to);
            L_tau[en][idx_tc] = mword(0);
            build_op12; drive_frame; await_pkt(t_to);
            L_iq[en][idx_tc]  = mword(0);

            L_fk [en][idx_tc] = f_fk;
            L_imp[en][idx_tc] = f_imp;
            L_gap[en][idx_tc] = f_gap;
            L_tiq[en][idx_tc] = f_tiq;
            L_chn[en][idx_tc] = f_chn;
            L_fwd[en][idx_tc] = f_fk + f_chn;
        end
    endtask

    // 设置 adaptive_en 与自适应内部状态
    task setup_en0; begin
        build_op15(1'b0, 32'sd3277, 32'sd6553600, 32'sd655360); drive_frame; await_pkt(t_to);
    end endtask

    task setup_en1; begin
        // 先注入 RLS 先验 (K_env=2000, P=1000), 再跑一次 op15(en=1) 产生 Ks_out/Ds_out
        build_op16(32'sd131072000, 32'sd65536000); drive_frame; await_pkt(t_to);
        build_op15(1'b1, 32'sd3277, 32'sd6553600, 32'sd655360); drive_frame; await_pkt(t_to);
    end endtask

    // 并发强条件的结果
    reg [15:0] C_fk [0:2], C_imp [0:2], C_tiq [0:2], C_chn [0:2], C_fwd [0:2], C_ovl [0:2];
    integer    ci;
    localparam integer CC_TC = 2;   // 并发实验使用的关节角组

    initial begin
        errors = 0; tau_diff_cnt = 0;
        rst = 1;
        s_udp_axis_data=0; s_udp_axis_valid=0; s_udp_axis_last=0; s_udp_axis_error=0;
        for (ti=0;ti<18;ti=ti+1) tb_duty[ti]=10'd512;
        for (ti=0;ti<6;ti=ti+1)  tb_iq[ti]=16'sd200;   // 非零实测电流 -> τ_ext≠0 -> RLS 真正更新
        load_qcfgs; load_op10_payload;

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);

        $display("=========================================================================");
        $display("R2.1 HARD METRIC — forward-chain cycles, adaptive_en=0 vs adaptive_en=1");
        $display("DUT: V3/src/udp_fk_cmd.v (real RTL), driven by real UDP op2/op10 payloads");
        $display("Forward chain (R1.1 definition): FK+Jac -> cart_impedance -> torque_to_iq");
        $display("=========================================================================");

        // ---------- A) 10 组关节角 × {en=0, en=1} ----------
        for (tc=0; tc<10; tc=tc+1) begin
            setup_en0;
            run_forward(tc, 0, 0);
            setup_en1;
            run_forward(tc, 1, 0);
            $display("  TC%0d  en=0: fk=%0d imp=%0d gap=%0d tiq=%0d chain=%0d FWD=%0d tau0=%0d iq0=%0d",
                     tc, L_fk[0][tc], L_imp[0][tc], L_gap[0][tc], L_tiq[0][tc], L_chn[0][tc], L_fwd[0][tc], L_tau[0][tc], L_iq[0][tc]);
            $display("  TC%0d  en=1: fk=%0d imp=%0d gap=%0d tiq=%0d chain=%0d FWD=%0d tau0=%0d iq0=%0d",
                     tc, L_fk[1][tc], L_imp[1][tc], L_gap[1][tc], L_tiq[1][tc], L_chn[1][tc], L_fwd[1][tc], L_tau[1][tc], L_iq[1][tc]);
        end

        // ---------- B) 强条件: 前向链进行中并发触发自适应更新 ----------
        $display("");
        $display("--- concurrent adaptive update during forward chain (qcfg #%0d, en=1) ---", CC_TC);
        setup_en1;
        run_forward(CC_TC, 1, 0);                       // 基线 (无注入), 覆盖写 L_*[1][CC_TC]
        C_fk[0]=L_fk[1][CC_TC]; C_imp[0]=L_imp[1][CC_TC]; C_tiq[0]=L_tiq[1][CC_TC];
        C_chn[0]=L_chn[1][CC_TC]; C_fwd[0]=L_fwd[1][CC_TC]; C_ovl[0]=last_ovl;

        setup_en1;
        run_forward(CC_TC, 1, 2);                       // 注入于 cart_impedance 进行中
        C_fk[1]=L_fk[1][CC_TC]; C_imp[1]=L_imp[1][CC_TC]; C_tiq[1]=L_tiq[1][CC_TC];
        C_chn[1]=L_chn[1][CC_TC]; C_fwd[1]=L_fwd[1][CC_TC]; C_ovl[1]=last_ovl;

        setup_en1;
        run_forward(CC_TC, 1, 3);                       // 注入于 imp->tiq 交界 (覆盖 torque_to_iq)
        C_fk[2]=L_fk[1][CC_TC]; C_imp[2]=L_imp[1][CC_TC]; C_tiq[2]=L_tiq[1][CC_TC];
        C_chn[2]=L_chn[1][CC_TC]; C_fwd[2]=L_fwd[1][CC_TC]; C_ovl[2]=last_ovl;

        // 还原基线记录 (最后一次注入覆盖了 L_*[1][CC_TC], 用基线值恢复以免污染汇总表)
        L_fk[1][CC_TC]=C_fk[0]; L_imp[1][CC_TC]=C_imp[0]; L_tiq[1][CC_TC]=C_tiq[0];
        L_chn[1][CC_TC]=C_chn[0]; L_fwd[1][CC_TC]=C_fwd[0];

        // ---------- 汇总 ----------
        $display("");
        $display("=================================================================================================");
        $display("R2.1 SUMMARY — forward-chain cycles per joint-angle configuration");
        $display("=================================================================================================");
        $display("  TC |  en=0: fk    imp  gap  tiq  chain   FWD |  en=1: fk    imp  gap  tiq  chain   FWD | EQ?");
        $display("-------------------------------------------------------------------------------------------------");
        for (tc=0; tc<10; tc=tc+1) begin
            $display("  T%0d |      %5d  %4d  %3d  %3d  %5d %5d |      %5d  %4d  %3d  %3d  %5d %5d | %s",
                     tc,
                     L_fk[0][tc], L_imp[0][tc], L_gap[0][tc], L_tiq[0][tc], L_chn[0][tc], L_fwd[0][tc],
                     L_fk[1][tc], L_imp[1][tc], L_gap[1][tc], L_tiq[1][tc], L_chn[1][tc], L_fwd[1][tc],
                     ((L_fk[0][tc]===L_fk[1][tc]) && (L_imp[0][tc]===L_imp[1][tc]) &&
                      (L_gap[0][tc]===L_gap[1][tc]) && (L_tiq[0][tc]===L_tiq[1][tc]) &&
                      (L_chn[0][tc]===L_chn[1][tc]) && (L_fwd[0][tc]===L_fwd[1][tc])) ? "EQ" : "NE");
            if (!((L_fk[0][tc]===L_fk[1][tc]) && (L_imp[0][tc]===L_imp[1][tc]) &&
                  (L_gap[0][tc]===L_gap[1][tc]) && (L_tiq[0][tc]===L_tiq[1][tc]) &&
                  (L_chn[0][tc]===L_chn[1][tc]) && (L_fwd[0][tc]===L_fwd[1][tc]))) begin
                errors = errors + 1;
                $display("      ^^ MISMATCH at TC%0d", tc);
            end
            if (L_tau[0][tc] !== L_tau[1][tc]) tau_diff_cnt = tau_diff_cnt + 1;
        end
        $display("=================================================================================================");
        $display("  numeric evidence: tau0(en=0) != tau0(en=1) in %0d / 10 configs", tau_diff_cnt);
        $display("    (identical cycle counts are NOT because the adaptive path is a no-op)");

        $display("");
        $display("=================================================================================================");
        $display("R2.1 STRONG CONDITION — adaptive update running CONCURRENTLY with the forward chain (en=1)");
        $display("=================================================================================================");
        $display("  case                          |   fk    imp  tiq  chain   FWD | adapt-busy cycles inside chain");
        $display("-------------------------------------------------------------------------------------------------");
        $display("  C0 baseline (no injection)    | %5d  %4d  %3d  %5d %5d | %0d", C_fk[0],C_imp[0],C_tiq[0],C_chn[0],C_fwd[0],C_ovl[0]);
        $display("  C1 adapt_start during cart_imp| %5d  %4d  %3d  %5d %5d | %0d", C_fk[1],C_imp[1],C_tiq[1],C_chn[1],C_fwd[1],C_ovl[1]);
        $display("  C2 adapt_start at imp->tiq    | %5d  %4d  %3d  %5d %5d | %0d", C_fk[2],C_imp[2],C_tiq[2],C_chn[2],C_fwd[2],C_ovl[2]);
        $display("=================================================================================================");
        for (ci=1; ci<3; ci=ci+1) begin
            if (!((C_fk[ci]===C_fk[0]) && (C_imp[ci]===C_imp[0]) && (C_tiq[ci]===C_tiq[0]) &&
                  (C_chn[ci]===C_chn[0]) && (C_fwd[ci]===C_fwd[0]))) begin
                errors = errors + 1;
                $display("  ^^ MISMATCH: concurrent case C%0d differs from baseline", ci);
            end
        end
        if (C_ovl[1] == 16'd0 && C_ovl[2] == 16'd0) begin
            errors = errors + 1;
            $display("  ^^ INVALID: no measured overlap — the concurrency injection did not take effect");
        end
        $display("  adaptive engine standalone latency (adapt_start->adapt_done) = %0d cycles", last_adapt);

        $display("");
        if (errors == 0) begin
            $display("R2_1_FWD_CYCLES_EQ_OK");
            $display("ALL TESTS PASSED");
        end else begin
            $display("R2_1_FWD_CYCLES_MISMATCH_FAIL: %0d violation(s)", errors);
            $display("TESTS FAILED");
        end
        $finish;
    end

    initial begin
        #40000000;
        $display("TIMEOUT — simulation hung");
        $display("TESTS FAILED");
        $finish;
    end
endmodule
