`timescale 1ns/1ps
// ============================================================================
// tb_r21_board.v — R2.1 板级验证 (host Part 5) 的 RTL 级理论期望值生成器。
// ----------------------------------------------------------------------------
// 板上流程 (E:\eth\old_host_test\host_pc_code\main.cpp Part 5) 与本 tb 逐字节对齐:
//   phase A (adaptive_en=0, 复位后从未发过 op15):
//     ... 20000 次 {op2(QV[n%32])+op10} ...          — 无状态泄漏, 尾链决定一切
//     op2(QV[0]) + op10(固定载荷) -> op11 读 τ = tauA — 应与 Part 4 期望 case0 一致
//   op15 (enable=1, q_err=1024×6, Ks_nom=9830400×6, Ds_nom=655360×6, 共 73B 载荷)
//     片上 τ_ext = Kt·iq_actual − τ_cmd; 板上 iq_actual≡0 (Part 2 写 adc=2048,
//     foc_top: Ia=ADCb+ADCc−2·ADCa=0 → clark/park 全零), 本 tb iq_act 绑 0 等价。
//     故 τ_ext = −tau_reg = −TAU_EXP[0] (tauA 链刚锁存)。
//   op17 读 72B = K_env_est0..5 + Ks_out0..5 + Ds_out0..5 (18×int32)
//   phase B (adaptive_en=1): 同样尾链 op2(QV[0])+op10 -> op11 读 τ = tauB
//   op18 (全程累积): 每条完整前向链拍数应恒 549 (R1.1 口径), 两态 min=max=549。
//
// 本 tb 每个 phase 跑 2 条链 (QV[31] 再 QV[0], 模拟板上循环尾序), 并在
//   phase A 后 / phase B 后各读一次 op18, 断言 last=min=max=549 (runs=2/4)。
// 产出 r21_board_expected.txt: 行1=op17 的 18 整数, 行2=tauB 6 整数, 行3=tauA 6 整数。
// 成功 sentinel: R21_BOARD_EXPECTED_OK
// 红线: 只例化真实 RTL (V3/src/udp_fk_cmd.v), 零改动; 期望值全部出自本仿真。
// ============================================================================
module tb_r21_board;
    localparam [7:0] ACK_BYTE = 8'hAA;
    localparam [31:0] CYC_MAGIC = 32'hE2E2E2E2;

    // ---- op15 载荷 (与 host Part 5 逐字节一致) ----
    localparam signed [31:0] P5_QE = 32'sd1024;      // q_err  = 0.015625 rad (Q16.16)
    localparam signed [31:0] P5_KN = 32'sd9830400;   // Ks_nom = 150.0 N/m
    localparam signed [31:0] P5_DN = 32'sd655360;    // Ds_nom = 10.0 N·s/m
    // 反比律 clamp 窗 (adaptive_impedance_v2 默认参数)
    localparam signed [31:0] KS_MIN = 32'sd655360;    // 10.0
    localparam signed [31:0] KS_MAX = 32'sd327680000; // 5000.0

    reg clk, rst;
    reg  [7:0] s_udp_axis_data; reg s_udp_axis_valid, s_udp_axis_last, s_udp_axis_error;
    wire [7:0] m_udp_axis_data; wire m_udp_axis_valid, m_udp_axis_last;
    wire [10:0] cfg_udp_len;
    reg  m_udp_axis_ready;

    reg [9:0]         tb_duty [0:17];   // 悬空输入, 恒 0
    reg signed [15:0] tb_iq   [0:5];    // iq_actual: 板上恒 0 (见文件头), 绑 0

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
        .phi_0(), .phi_1(), .phi_2(), .phi_3(), .phi_4(), .phi_5(),
        .adc_a_0(), .adc_b_0(), .adc_c_0(), .adc_a_1(), .adc_b_1(), .adc_c_1(),
        .adc_a_2(), .adc_b_2(), .adc_c_2(), .adc_a_3(), .adc_b_3(), .adc_c_3(),
        .adc_a_4(), .adc_b_4(), .adc_c_4(), .adc_a_5(), .adc_b_5(), .adc_c_5(),
        .iq_act_0(tb_iq[0]), .iq_act_1(tb_iq[1]), .iq_act_2(tb_iq[2]),
        .iq_act_3(tb_iq[3]), .iq_act_4(tb_iq[4]), .iq_act_5(tb_iq[5]));

    initial begin clk=0; forever #5 clk=~clk; end

    // ---- udp_tx 契约接收模型 (照抄 tb_udp_fk_cmd / tb_force_board) ----
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

    reg [7:0] fr [0:127]; integer frn;
    integer i, j, guard;
    reg t_to;

    task drive_frame; begin
        @(posedge clk);
        for (j=0;j<frn;j=j+1) begin
            s_udp_axis_data<=fr[j]; s_udp_axis_valid<=1'b1;
            s_udp_axis_last<=(j==frn-1); s_udp_axis_error<=1'b0;
            @(posedge clk);
        end
        s_udp_axis_valid<=0; s_udp_axis_last<=0; s_udp_axis_error<=0;
    end endtask

    task await_pkt; output to; begin
        to=0; guard=0;
        while (pkt_sent!==1'b1) begin @(posedge clk); guard=guard+1; if (guard>20000) begin to=1; disable await_pkt; end end
    end endtask

    function signed [31:0] mword; input integer n;
        mword = {mbuf[4*n+3], mbuf[4*n+2], mbuf[4*n+1], mbuf[4*n]};
    endfunction

    // ---- 关节角: QV[0] 与 QV[31] (与 host main.cpp QV 表逐字一致, Q16.16 raw) ----
    reg signed [31:0] Q0 [0:5];
    reg signed [31:0] Q31 [0:5];
    // Part 4 期望表 case 0 (host TAU_EXP[0], tb_force_board 产物) —— tauA 自检黄金值
    reg signed [31:0] TAU_EXP0 [0:5];
    initial begin
        Q0[0]=32'sd57412;   Q0[1]=-32'sd195589; Q0[2]=-32'sd92637;
        Q0[3]=-32'sd113975; Q0[4]=32'sd97373;   Q0[5]=32'sd72760;
        Q31[0]=32'sd6554;   Q31[1]=32'sd13107;  Q31[2]=-32'sd19661;
        Q31[3]=32'sd26214;  Q31[4]=32'sd7;      Q31[5]=-32'sd32768;
        TAU_EXP0[0]=32'sd1657026;  TAU_EXP0[1]=-32'sd1154870; TAU_EXP0[2]=-32'sd1458094;
        TAU_EXP0[3]=32'sd101190;   TAU_EXP0[4]=32'sd1898588;  TAU_EXP0[5]=32'sd921611;
    end

    // ---- op10 固定载荷 (与 host e2_build_op10 字节级一致) ----
    reg signed [31:0] XDv [0:5], Kv [0:5], Dv [0:5], XVv [0:5];
    initial begin
        XDv[0]=32'sd66;      XDv[1]=-32'sd66;     XDv[2]=32'sd66;
        XDv[3]=32'sd0;       XDv[4]=32'sd131;     XDv[5]=-32'sd66;
        Kv[0]=32'sd6553600;  Kv[1]=32'sd6553600;  Kv[2]=32'sd6553600;
        Kv[3]=32'sd655360;   Kv[4]=32'sd655360;   Kv[5]=32'sd655360;
        Dv[0]=32'sd655360;   Dv[1]=32'sd655360;   Dv[2]=32'sd655360;
        Dv[3]=32'sd65536;    Dv[4]=32'sd65536;    Dv[5]=32'sd65536;
        XVv[0]=0; XVv[1]=0; XVv[2]=0; XVv[3]=0; XVv[4]=0; XVv[5]=0;
    end

    reg signed [31:0] qv [0:5];
    task build_op2; begin       // [2][addr4][len=24 LE2][q0..q5] = 31B (同 host e2_build_op2)
        fr[0]=8'd2; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=qv[i][7:0];   fr[8+4*i]=qv[i][15:8];
            fr[9+4*i]=qv[i][23:16]; fr[10+4*i]=qv[i][31:24];
        end
        frn=31;
    end endtask

    task build_op10; begin      // [10][addr4][len=96 LE2][x_des][K][D][xv] = 103B (同 host e2_build_op10)
        fr[0]=8'd10; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd96; fr[6]=0;
        for (i=0;i<6;i=i+1) begin
            fr[7+4*i]=XDv[i][7:0];   fr[8+4*i]=XDv[i][15:8];   fr[9+4*i]=XDv[i][23:16];  fr[10+4*i]=XDv[i][31:24];
            fr[31+4*i]=Kv[i][7:0];   fr[32+4*i]=Kv[i][15:8];   fr[33+4*i]=Kv[i][23:16];  fr[34+4*i]=Kv[i][31:24];
            fr[55+4*i]=Dv[i][7:0];   fr[56+4*i]=Dv[i][15:8];   fr[57+4*i]=Dv[i][23:16];  fr[58+4*i]=Dv[i][31:24];
            fr[79+4*i]=XVv[i][7:0];  fr[80+4*i]=XVv[i][15:8];  fr[81+4*i]=XVv[i][23:16]; fr[82+4*i]=XVv[i][31:24];
        end
        frn=103;
    end endtask

    task build_op15; begin      // [15][addr4][len=73 LE2][enable][q_err×6][Ks_nom×6][Ds_nom×6] = 80B
        fr[0]=8'd15; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd73; fr[6]=0;
        fr[7]=8'd1;                                  // enable=1 -> adaptive_en 置位
        for (i=0;i<6;i=i+1) begin
            fr[8+4*i] =P5_QE[7:0];  fr[9+4*i] =P5_QE[15:8];  fr[10+4*i]=P5_QE[23:16]; fr[11+4*i]=P5_QE[31:24];
            fr[32+4*i]=P5_KN[7:0];  fr[33+4*i]=P5_KN[15:8];  fr[34+4*i]=P5_KN[23:16]; fr[35+4*i]=P5_KN[31:24];
            fr[56+4*i]=P5_DN[7:0];  fr[57+4*i]=P5_DN[15:8];  fr[58+4*i]=P5_DN[23:16]; fr[59+4*i]=P5_DN[31:24];
        end
        frn=80;
    end endtask

    task build_read; input [7:0] op; input [7:0] len; begin  // [op][addr4][len LE2] = 7B
        fr[0]=op; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=len; fr[6]=0; frn=7;
    end endtask

    integer errs;

    // 一条完整前向链: op2(qv) -> ACK; op10(固定载荷) -> ACK
    task run_chain; input [127:0] tag; begin
        build_op2; drive_frame; await_pkt(t_to);
        if (t_to || mbuf[0]!==ACK_BYTE || mlen!==11'd18) begin
            $display("ERROR %0s: op2 ACK bad (to=%0d mbuf0=%02x mlen=%0d)",tag,t_to,mbuf[0],mlen); errs=errs+1;
        end
        build_op10; drive_frame; await_pkt(t_to);
        if (t_to || mbuf[0]!==ACK_BYTE || mlen!==11'd18) begin
            $display("ERROR %0s: op10 ACK bad (to=%0d mbuf0=%02x mlen=%0d)",tag,t_to,mbuf[0],mlen); errs=errs+1;
        end
    end endtask

    // op18 读 + 断言 last=min=max=549, runs=exp_runs, 魔数正确
    reg [31:0] cycw [0:5];
    task check_op18; input [127:0] tag; input [31:0] exp_runs; begin
        build_read(8'd18, 8'd24); drive_frame; await_pkt(t_to);
        if (t_to || mlen!==11'd24) begin
            $display("ERROR %0s: op18 reply bad (to=%0d mlen=%0d)",tag,t_to,mlen); errs=errs+1;
        end
        for (i=0;i<6;i=i+1) begin
            cycw[i] = mword(i);
            if (^cycw[i] === 1'bx) begin $display("ERROR %0s: op18 word %0d has X",tag,i); errs=errs+1; end
        end
        $display("%0s: op18 last=%0d min=%0d max=%0d runs=%0d magic=%08x",
                 tag, cycw[0], cycw[1], cycw[2], cycw[3], cycw[4]);
        if (cycw[4] !== CYC_MAGIC) begin $display("ERROR %0s: op18 magic bad",tag); errs=errs+1; end
        if (cycw[0] !== 32'd549 || cycw[1] !== 32'd549 || cycw[2] !== 32'd549) begin
            $display("ERROR %0s: forward-chain cycles not 549/549/549 (last=%0d min=%0d max=%0d)",
                     tag, cycw[0], cycw[1], cycw[2]);
            errs=errs+1;
        end
        if (cycw[3] !== exp_runs) begin
            $display("ERROR %0s: op18 runs=%0d expect %0d",tag,cycw[3],exp_runs); errs=errs+1;
        end
    end endtask

    integer fd, k, diffs;
    reg signed [31:0] tauA [0:5];
    reg signed [31:0] tauB [0:5];
    reg signed [31:0] adaptw [0:17];   // op17: K_env_est×6, Ks_out×6, Ds_out×6

    initial begin
        errs = 0;
        s_udp_axis_data=0; s_udp_axis_valid=0; s_udp_axis_last=0; s_udp_axis_error=0;
        for (i=0;i<18;i=i+1) tb_duty[i]=10'd0;
        for (i=0;i<6;i=i+1)  tb_iq[i]=16'sd0;
        rst=1; repeat(12) @(posedge clk); rst=0; repeat(5) @(posedge clk);

        // ================= phase A: adaptive_en=0 =================
        for (i=0;i<6;i=i+1) qv[i]=Q31[i]; run_chain("phaseA chain(QV31)");
        for (i=0;i<6;i=i+1) qv[i]=Q0[i];  run_chain("phaseA chain(QV0)");

        build_read(8'd11, 8'd24); drive_frame; await_pkt(t_to);        // op11 -> tauA
        if (t_to || mlen!==11'd24) begin
            $display("ERROR phaseA: op11 reply bad (to=%0d mlen=%0d)",t_to,mlen); errs=errs+1;
        end
        for (k=0;k<6;k=k+1) begin
            tauA[k] = mword(k);
            if (^tauA[k] === 1'bx) begin $display("ERROR phaseA: tauA[%0d] has X",k); errs=errs+1; end
            // 自检: 必须与 Part 4 期望表 case 0 逐位一致 (否则状态泄漏或载荷不一致)
            if (tauA[k] !== TAU_EXP0[k]) begin
                $display("ERROR phaseA SELF-CHECK: tauA[%0d]=%0d != TAU_EXP0[%0d]=%0d",k,tauA[k],k,TAU_EXP0[k]);
                errs=errs+1;
            end
        end
        $display("phaseA tauA = %0d %0d %0d %0d %0d %0d (self-check vs Part4 case0 %s)",
                 tauA[0],tauA[1],tauA[2],tauA[3],tauA[4],tauA[5], (errs==0)?"MATCH":"CHECK-LOG");

        check_op18("phaseA", 32'd2);

        // ================= op15: enable=1 =================
        build_op15; drive_frame; await_pkt(t_to);
        if (t_to || mbuf[0]!==ACK_BYTE || mlen!==11'd18) begin
            $display("ERROR op15: ACK bad (to=%0d mbuf0=%02x mlen=%0d)",t_to,mbuf[0],mlen); errs=errs+1;
        end

        // ================= op17: 读 18 个自适应输出 =================
        build_read(8'd17, 8'd72); drive_frame; await_pkt(t_to);
        if (t_to || mlen!==11'd72) begin
            $display("ERROR op17: reply bad (to=%0d mlen=%0d)",t_to,mlen); errs=errs+1;
        end
        for (k=0;k<18;k=k+1) begin
            adaptw[k] = mword(k);
            if (^adaptw[k] === 1'bx) begin $display("ERROR op17: word %0d has X",k); errs=errs+1; end
        end
        $display("op17 K_env_est = %0d %0d %0d %0d %0d %0d",
                 adaptw[0],adaptw[1],adaptw[2],adaptw[3],adaptw[4],adaptw[5]);
        $display("op17 Ks_out    = %0d %0d %0d %0d %0d %0d",
                 adaptw[6],adaptw[7],adaptw[8],adaptw[9],adaptw[10],adaptw[11]);
        $display("op17 Ds_out    = %0d %0d %0d %0d %0d %0d",
                 adaptw[12],adaptw[13],adaptw[14],adaptw[15],adaptw[16],adaptw[17]);
        // 工作点检查: Ks_out 须严格落在 (KS_MIN, KS_MAX) 开区间 (未触 clamp, 非平凡),
        // 且与 op10 下发的 K 不同 (证明 phase B 的 mux 确实换了增益)
        for (k=0;k<6;k=k+1) begin
            if (!(adaptw[6+k] > KS_MIN && adaptw[6+k] < KS_MAX)) begin
                $display("ERROR op17: Ks_out[%0d]=%0d clamped or out of (%0d,%0d)",k,adaptw[6+k],KS_MIN,KS_MAX);
                errs=errs+1;
            end
            if (adaptw[6+k] === Kv[k]) begin
                $display("ERROR op17: Ks_out[%0d]=%0d equals op10 K -> not distinguishable",k,adaptw[6+k]);
                errs=errs+1;
            end
        end

        // ================= phase B: adaptive_en=1 =================
        for (i=0;i<6;i=i+1) qv[i]=Q31[i]; run_chain("phaseB chain(QV31)");
        for (i=0;i<6;i=i+1) qv[i]=Q0[i];  run_chain("phaseB chain(QV0)");

        build_read(8'd11, 8'd24); drive_frame; await_pkt(t_to);        // op11 -> tauB
        if (t_to || mlen!==11'd24) begin
            $display("ERROR phaseB: op11 reply bad (to=%0d mlen=%0d)",t_to,mlen); errs=errs+1;
        end
        diffs=0;
        for (k=0;k<6;k=k+1) begin
            tauB[k] = mword(k);
            if (^tauB[k] === 1'bx) begin $display("ERROR phaseB: tauB[%0d] has X",k); errs=errs+1; end
            if (tauB[k] !== tauA[k]) diffs=diffs+1;
        end
        $display("phaseB tauB = %0d %0d %0d %0d %0d %0d (%0d/6 differ from tauA)",
                 tauB[0],tauB[1],tauB[2],tauB[3],tauB[4],tauB[5], diffs);
        if (diffs==0) begin
            $display("ERROR phaseB: tauB identical to tauA -> adaptation had no effect"); errs=errs+1;
        end

        check_op18("phaseB", 32'd4);

        // ================= 期望值文件 =================
        fd = $fopen("r21_board_expected.txt","w");
        if (fd==0) begin $display("FATAL: cannot open r21_board_expected.txt"); $finish; end
        $fdisplay(fd,"%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
            adaptw[0],adaptw[1],adaptw[2],adaptw[3],adaptw[4],adaptw[5],
            adaptw[6],adaptw[7],adaptw[8],adaptw[9],adaptw[10],adaptw[11],
            adaptw[12],adaptw[13],adaptw[14],adaptw[15],adaptw[16],adaptw[17]);
        $fdisplay(fd,"%0d %0d %0d %0d %0d %0d", tauB[0],tauB[1],tauB[2],tauB[3],tauB[4],tauB[5]);
        $fdisplay(fd,"%0d %0d %0d %0d %0d %0d", tauA[0],tauA[1],tauA[2],tauA[3],tauA[4],tauA[5]);
        $fclose(fd);

        if (errs==0) $display("R21_BOARD_EXPECTED_OK");
        else         $display("R21_BOARD_EXPECTED_FAILED (%0d errors)", errs);
        $finish;
    end

    initial begin
        #10000000;
        $display("TIMEOUT - simulation hung");
        $display("R21_BOARD_EXPECTED_FAILED (timeout)");
        $finish;
    end
endmodule
