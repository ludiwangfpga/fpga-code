`timescale 1ns/1ps
// tb_force_board — 力链板上扫描 (Table VI 载体升级) 的 RTL 级理论期望值生成器。
//
// 32 组关节角与 host (E:\eth\old_host_test\host_pc_code\main.cpp) 的 QV[32][6] 逐字相同
// (前 28 例 seed42 随机 + 4 特殊构型, 其 FK pose 已板级逐位验证)。逐例执行:
//   op2(q[n]) -> 等 18B ACK (FK+雅可比锁存, 用的是舍入版 fk_engine)
//   op10(固定载荷, 与 host e2_build_op10 字节级一致) -> 等 18B ACK (τ 与 iq 已锁存)
//   op11 读 τ  -> 24B = 6×int32 Q16.16
//   op12 读 iq -> 24B = 6×int32 (16 位 iq 符号扩展)
// 把每例 6×τ_q16 + 6×iq 共 12 个原始整数写入 force_board_expected.txt (32 行×12 列)。
//
// op10 固定载荷语义: RTL (wrench_calc.v L31) Δx = x_des − pose, 即 x_des 是**绝对**期望
// 位姿。本扫描用固定绝对 x_des = {66,-66,66,0,131,-66} (近零位姿), Δx=x_des−pose 会很大、
// τ 大/iq 多饱和 ±2000 —— 对拍只看逐位一致, 数值大小无所谓; 载荷固定、简单、可复现。
//   K = {6553600,6553600,6553600,655360,655360,655360}   (100/100/100/10/10/10 Q16.16)
//   D = {655360,655360,655360,65536,65536,65536}         (10/10/10/1/1/1 Q16.16)
//   ẋ = 0
//
// TX 侧挂 tb_udp_fk_cmd 同款「忠实复刻 udp_tx 契约」的接收模型 (M_IDLE/M_WRITE/M_DONE)。
// 成功 sentinel: FORCE_BOARD_EXPECTED_OK
module tb_force_board;
    localparam [7:0] ACK_BYTE = 8'hAA;

    reg clk, rst;
    reg  [7:0] s_udp_axis_data; reg s_udp_axis_valid, s_udp_axis_last, s_udp_axis_error;
    wire [7:0] m_udp_axis_data; wire m_udp_axis_valid, m_udp_axis_last;
    wire [10:0] cfg_udp_len;
    reg  m_udp_axis_ready;

    reg [9:0]         tb_duty [0:17];   // op13 输入 (本 tb 不用, 恒 0)
    reg signed [15:0] tb_iq   [0:5];    // iq_actual 输入 (本 tb 不用, 恒 0; 不发 op15, adaptive_en 恒 0)

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

    // ---- udp_tx 契约接收模型 (照抄 tb_udp_fk_cmd) ----
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
    integer j, guard;
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

    // ---- 32 组关节角 (与 host main.cpp QV[32][6] 逐字一致, Q16.16 raw) ----
    reg signed [31:0] QV [0:191];
    integer i;
    initial begin
        QV[0]=57412;    QV[1]=-195589;  QV[2]=-92637;   QV[3]=-113975;  QV[4]=97373;    QV[5]=72760;
        QV[6]=161490;   QV[7]=-170088;  QV[8]=-32151;   QV[9]=-193618;  QV[10]=-115858; QV[11]=2205;
        QV[12]=-194961; QV[13]=-124011; QV[14]=61719;   QV[15]=18506;   QV[16]=-115116; QV[17]=36757;
        QV[18]=127416;  QV[19]=-203211; QV[20]=125929;  QV[21]=81589;   QV[22]=-65781;  QV[23]=-141865;
        QV[24]=188269;  QV[25]=-67286;  QV[26]=-167697; QV[27]=-166062; QV[28]=143089;  QV[29]=42712;
        QV[30]=126468;  QV[31]=94598;   QV[32]=14918;   QV[33]=194817;  QV[34]=-50016;  QV[35]=21429;
        QV[36]=135641;  QV[37]=48803;   QV[38]=148942;  QV[39]=31852;   QV[40]=84238;   QV[41]=-187018;
        QV[42]=-112045; QV[43]=-86725;  QV[44]=-173031; QV[45]=-110030; QV[46]=-164298; QV[47]=-91425;
        QV[48]=55871;   QV[49]=-55659;  QV[50]=-53456;  QV[51]=-119618; QV[52]=-95953;  QV[53]=179803;
        QV[54]=60957;   QV[55]=44937;   QV[56]=-135417; QV[57]=94349;   QV[58]=-138602; QV[59]=-49637;
        QV[60]=201573;  QV[61]=57648;   QV[62]=23450;   QV[63]=76020;   QV[64]=141178;  QV[65]=113650;
        QV[66]=-111571; QV[67]=-192669; QV[68]=-75992;  QV[69]=-95638;  QV[70]=-119010; QV[71]=182379;
        QV[72]=154979;  QV[73]=-76311;  QV[74]=64006;   QV[75]=-42976;  QV[76]=170700;  QV[77]=-16944;
        QV[78]=-96816;  QV[79]=-104332; QV[80]=25270;   QV[81]=-97697;  QV[82]=34830;   QV[83]=163813;
        QV[84]=-41424;  QV[85]=-115577; QV[86]=204873;  QV[87]=3923;    QV[88]=-168453; QV[89]=-186486;
        QV[90]=-160737; QV[91]=52479;   QV[92]=120271;  QV[93]=-32053;  QV[94]=-179728; QV[95]=-48746;
        QV[96]=204290;  QV[97]=11989;   QV[98]=193978;  QV[99]=148560;  QV[100]=-201160;QV[101]=90888;
        QV[102]=74824;  QV[103]=15223;  QV[104]=-96016; QV[105]=58045;  QV[106]=-159953;QV[107]=-26862;
        QV[108]=-19055; QV[109]=186870; QV[110]=154767; QV[111]=-97430; QV[112]=241;    QV[113]=-132323;
        QV[114]=169910; QV[115]=152570; QV[116]=-82995; QV[117]=57216;  QV[118]=44871;  QV[119]=-142952;
        QV[120]=108095; QV[121]=16215;  QV[122]=114731; QV[123]=12499;  QV[124]=-205652;QV[125]=-72408;
        QV[126]=-197867;QV[127]=176692; QV[128]=155948; QV[129]=136572; QV[130]=-79261; QV[131]=-182035;
        QV[132]=155655; QV[133]=184043; QV[134]=-170617;QV[135]=-5769;  QV[136]=-177387;QV[137]=107309;
        QV[138]=109464; QV[139]=-153019;QV[140]=-10178; QV[141]=20508;  QV[142]=-96744; QV[143]=153359;
        QV[144]=-31650; QV[145]=-118674;QV[146]=16181;  QV[147]=94680;  QV[148]=-123058;QV[149]=-77530;
        QV[150]=203890; QV[151]=61716;  QV[152]=-25489; QV[153]=7237;   QV[154]=-156061;QV[155]=-113363;
        QV[156]=-66672; QV[157]=36363;  QV[158]=-111132;QV[159]=-115207;QV[160]=-176654;QV[161]=53985;
        QV[162]=-111615;QV[163]=166942; QV[164]=148089; QV[165]=-176710;QV[166]=-107883;QV[167]=69581;
        QV[168]=0;      QV[169]=0;      QV[170]=0;      QV[171]=0;      QV[172]=0;      QV[173]=0;
        QV[174]=102944; QV[175]=102944; QV[176]=102944; QV[177]=102944; QV[178]=102944; QV[179]=102944;
        QV[180]=205887; QV[181]=-205887;QV[182]=205887; QV[183]=-205887;QV[184]=205887; QV[185]=-205887;
        QV[186]=6554;   QV[187]=13107;  QV[188]=-19661; QV[189]=26214;  QV[190]=7;      QV[191]=-32768;
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

    task build_read6; input [7:0] op; begin  // [op][addr4][len=24 LE2] = 7B (op11/op12)
        fr[0]=op; fr[1]=0; fr[2]=0; fr[3]=0; fr[4]=0; fr[5]=8'd24; fr[6]=0; frn=7;
    end endtask

    integer fd, n, k, errs;
    reg signed [31:0] tau_c [0:5];
    reg signed [31:0] iq_c  [0:5];
    reg case_bad;

    initial begin
        errs = 0;
        s_udp_axis_data=0; s_udp_axis_valid=0; s_udp_axis_last=0; s_udp_axis_error=0;
        for (i=0;i<18;i=i+1) tb_duty[i]=10'd0;
        for (i=0;i<6;i=i+1)  tb_iq[i]=16'sd0;
        rst=1; repeat(12) @(posedge clk); rst=0; repeat(5) @(posedge clk);

        fd = $fopen("force_board_expected.txt","w");
        if (fd==0) begin $display("FATAL: cannot open force_board_expected.txt"); $finish; end

        for (n=0;n<32;n=n+1) begin
            case_bad = 0;
            for (i=0;i<6;i=i+1) qv[i] = QV[6*n+i];

            build_op2; drive_frame; await_pkt(t_to);                 // op2 -> 18B ACK
            if (t_to || mbuf[0]!==ACK_BYTE || mlen!==11'd18) begin
                $display("ERROR case %0d: op2 ACK bad (to=%0d mbuf0=%02x mlen=%0d)",n,t_to,mbuf[0],mlen);
                case_bad=1;
            end

            build_op10; drive_frame; await_pkt(t_to);                // op10 -> 18B ACK
            if (t_to || mbuf[0]!==ACK_BYTE || mlen!==11'd18) begin
                $display("ERROR case %0d: op10 ACK bad (to=%0d mbuf0=%02x mlen=%0d)",n,t_to,mbuf[0],mlen);
                case_bad=1;
            end

            build_read6(8'd11); drive_frame; await_pkt(t_to);        // op11 -> 24B tau
            if (t_to || mlen!==11'd24) begin
                $display("ERROR case %0d: op11 reply bad (to=%0d mlen=%0d)",n,t_to,mlen); case_bad=1;
            end
            for (k=0;k<6;k=k+1) begin
                tau_c[k] = mword(k);
                if (^tau_c[k] === 1'bx) begin $display("ERROR case %0d: tau[%0d] has X",n,k); case_bad=1; end
            end

            build_read6(8'd12); drive_frame; await_pkt(t_to);        // op12 -> 24B iq
            if (t_to || mlen!==11'd24) begin
                $display("ERROR case %0d: op12 reply bad (to=%0d mlen=%0d)",n,t_to,mlen); case_bad=1;
            end
            for (k=0;k<6;k=k+1) begin
                iq_c[k] = mword(k);
                if (^iq_c[k] === 1'bx) begin $display("ERROR case %0d: iq[%0d] has X",n,k); case_bad=1; end
            end

            if (case_bad) errs = errs + 1;
            $fdisplay(fd,"%0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                tau_c[0],tau_c[1],tau_c[2],tau_c[3],tau_c[4],tau_c[5],
                iq_c[0],iq_c[1],iq_c[2],iq_c[3],iq_c[4],iq_c[5]);
            $display("case %0d: tau=(%0d,%0d,%0d,%0d,%0d,%0d) iq=(%0d,%0d,%0d,%0d,%0d,%0d)",
                n, tau_c[0],tau_c[1],tau_c[2],tau_c[3],tau_c[4],tau_c[5],
                   iq_c[0],iq_c[1],iq_c[2],iq_c[3],iq_c[4],iq_c[5]);
        end

        $fclose(fd);
        if (errs==0) $display("FORCE_BOARD_EXPECTED_OK");
        else         $display("FORCE_BOARD_EXPECTED_FAILED (%0d bad cases)", errs);
        $finish;
    end
endmodule
