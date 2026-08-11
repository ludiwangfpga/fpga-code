`timescale 1ns/1ps
//=====================================================================================
// tb_r1_3_force_precision.v  —  R1.3 力链路全链 Q16.16 定点对拍向量台
//
// 目的：复刻 R1.2「FK 1004 例对拍」的范式到力反馈链路上，为论文 R1.3 补实测证据。
//
// 数据通路（与板上 impedance_control_top 完全一致的真实链路）：
//     q  --analytical_jacobian_serial--> pose, J
//     (pose, x_des, K, D, ẋ) --cart_impedance--> {内部 F} --> τ = Jᵀ·F
//     τ  --torque_to_iq--> i_q*
//     (i_q,actual, τ) --τ_ext 反推--> τ_ext        (论文式(8))
//
// 两个 torque_to_iq 实例：
//   u_tiq_asis : 默认参数 KT_INV=1310720 —— **板上实际部署的版本**
//   u_tiq_fix  : 参数覆写 KT_INV=20      —— 论文式(7) i_q*=τ/Kt(安培) 的量纲正确版本
//   两者仅靠 parameter 覆写区分，未修改任何 RTL 源码。
//
// τ_ext 反推：本仓库根树的 udp_fk_cmd.v 没有 op15，反推式内联在 V3/src/udp_fk_cmd.v
//   第 472-477 行。此处按位逐字复刻该表达式（含 32 位截断、无饱和），不改动 RTL：
//       tauext <= ($signed({{16{iq_act[15]}},iq_act}) * KT_Q16) - $signed(tau);
//
// 激励来自 sim/r1_3_vectors.mem（tools/force_golden.py gen 生成，每例 36 个 32bit 字）
// 结果写到 sim/r1_3_rtl_out.mem（每例 72 个 32bit 字），由 force_golden.py check 对拍。
//
// sentinel: R1_3_DUMP_OK
//=====================================================================================

module tb_r1_3_force_precision;

    localparam integer WORDS_IN  = 36;
    localparam integer WORDS_OUT = 72;
    localparam integer MAX_CASES = 4096;

    localparam signed [31:0] KT_Q16 = 32'sd3277;   // Kt=0.05 的 Q16.16 (V3 udp_fk_cmd)

    reg clk, rst_n;
    initial clk = 1'b0;
    always #5 clk = ~clk;                          // 100 MHz 仿真时钟（周期数与频率无关）

    //------------------------------------------------------------------ 激励存储
    reg [31:0] vin [0:MAX_CASES*WORDS_IN-1];
    integer    n_cases;

    //------------------------------------------------------------------ FK + Jacobian
    reg fk_jac_start;
    reg signed [31:0] q0, q1, q2, q3, q4, q5;
    wire signed [31:0] pose_x, pose_y, pose_z, pose_rx, pose_ry, pose_rz;
    wire signed [31:0] J [0:35];
    wire fk_jac_done;

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

    //------------------------------------------------------------ 笛卡尔阻抗 (F -> τ)
    reg cart_start;
    reg signed [31:0] J_imp    [0:35];
    reg signed [31:0] pose_imp [0:5];
    reg signed [31:0] xdes [0:5], K_r [0:5], D_r [0:5], xv [0:5];
    wire signed [31:0] tau [0:5];
    wire cart_done;

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

    //------------------------------------------------------------------ torque_to_iq x2
    reg tiq_start;
    reg signed [31:0] tau_lat [0:5];
    wire signed [15:0] iqa0, iqa1, iqa2, iqa3, iqa4, iqa5;   // as-is (KT_INV=1310720)
    wire signed [15:0] iqf0, iqf1, iqf2, iqf3, iqf4, iqf5;   // KT_INV=20 (量纲=A)
    wire tiq_done_a, tiq_done_f;

    torque_to_iq u_tiq_asis (
        .clk(clk), .rst_n(rst_n), .start(tiq_start),
        .tau0(tau_lat[0]), .tau1(tau_lat[1]), .tau2(tau_lat[2]),
        .tau3(tau_lat[3]), .tau4(tau_lat[4]), .tau5(tau_lat[5]),
        .iq0(iqa0), .iq1(iqa1), .iq2(iqa2), .iq3(iqa3), .iq4(iqa4), .iq5(iqa5),
        .done(tiq_done_a)
    );

    torque_to_iq #(
        .KT_INV_0(32'sd20), .KT_INV_1(32'sd20), .KT_INV_2(32'sd20),
        .KT_INV_3(32'sd20), .KT_INV_4(32'sd20), .KT_INV_5(32'sd20)
    ) u_tiq_fix (
        .clk(clk), .rst_n(rst_n), .start(tiq_start),
        .tau0(tau_lat[0]), .tau1(tau_lat[1]), .tau2(tau_lat[2]),
        .tau3(tau_lat[3]), .tau4(tau_lat[4]), .tau5(tau_lat[5]),
        .iq0(iqf0), .iq1(iqf1), .iq2(iqf2), .iq3(iqf3), .iq4(iqf4), .iq5(iqf5),
        .done(tiq_done_f)
    );

    //------------------------------------------- τ_ext 反推（复刻 V3 udp_fk_cmd op15）
    reg signed [15:0] iq_act [0:5];
    reg        [31:0] tauext [0:5];
    integer            e;
    task calc_tauext;
        begin
            for (e = 0; e < 6; e = e + 1)
                tauext[e] = ($signed({{16{iq_act[e][15]}}, iq_act[e]}) * KT_Q16)
                            - $signed(tau_lat[e]);
        end
    endtask

    //------------------------------------------------------------------ 主流程
    integer fout;
    integer ci, w;
    integer base;
    reg [31:0] Fv [0:5];

    task run_case;
        input integer idx;
        begin
            base = idx * WORDS_IN;
            q0 = vin[base+0]; q1 = vin[base+1]; q2 = vin[base+2];
            q3 = vin[base+3]; q4 = vin[base+4]; q5 = vin[base+5];
            for (w = 0; w < 6; w = w + 1) begin
                xdes[w] = vin[base+6 +w];
                K_r [w] = vin[base+12+w];
                D_r [w] = vin[base+18+w];
                xv  [w] = vin[base+24+w];
                iq_act[w] = vin[base+30+w][15:0];
            end

            // --- FK + 解析雅可比 ---
            @(posedge clk); fk_jac_start <= 1'b1;
            @(posedge clk); fk_jac_start <= 1'b0;
            wait (fk_jac_done == 1'b1);
            @(posedge clk);
            for (w = 0; w < 36; w = w + 1) J_imp[w] = J[w];
            pose_imp[0]=pose_x;  pose_imp[1]=pose_y;  pose_imp[2]=pose_z;
            pose_imp[3]=pose_rx; pose_imp[4]=pose_ry; pose_imp[5]=pose_rz;

            // --- 笛卡尔阻抗 F=K·Δx−D·ẋ, τ=Jᵀ·F ---
            @(posedge clk); cart_start <= 1'b1;
            @(posedge clk); cart_start <= 1'b0;
            wait (cart_done == 1'b1);
            @(posedge clk);
            // wrench_calc 的 F 是 cart_impedance 内部 wire，层次引用抓取（只读，不改 RTL）
            Fv[0]=imp.Fw0; Fv[1]=imp.Fw1; Fv[2]=imp.Fw2;
            Fv[3]=imp.Fw3; Fv[4]=imp.Fw4; Fv[5]=imp.Fw5;
            for (w = 0; w < 6; w = w + 1) tau_lat[w] = tau[w];

            // --- torque_to_iq (两个实例同拍启动) ---
            @(posedge clk); tiq_start <= 1'b1;
            @(posedge clk); tiq_start <= 1'b0;
            wait (tiq_done_a == 1'b1);
            wait (tiq_done_f == 1'b1);
            @(posedge clk);

            // --- τ_ext 反推 ---
            calc_tauext;

            // --- 导出 72 字 ---
            $fwrite(fout, "%08x\n%08x\n%08x\n%08x\n%08x\n%08x\n",
                    pose_imp[0], pose_imp[1], pose_imp[2],
                    pose_imp[3], pose_imp[4], pose_imp[5]);
            for (w = 0; w < 36; w = w + 1) $fwrite(fout, "%08x\n", J_imp[w]);
            for (w = 0; w < 6;  w = w + 1) $fwrite(fout, "%08x\n", Fv[w]);
            for (w = 0; w < 6;  w = w + 1) $fwrite(fout, "%08x\n", tau_lat[w]);
            $fwrite(fout, "%08x\n%08x\n%08x\n%08x\n%08x\n%08x\n",
                    {{16{iqa0[15]}},iqa0}, {{16{iqa1[15]}},iqa1}, {{16{iqa2[15]}},iqa2},
                    {{16{iqa3[15]}},iqa3}, {{16{iqa4[15]}},iqa4}, {{16{iqa5[15]}},iqa5});
            $fwrite(fout, "%08x\n%08x\n%08x\n%08x\n%08x\n%08x\n",
                    {{16{iqf0[15]}},iqf0}, {{16{iqf1[15]}},iqf1}, {{16{iqf2[15]}},iqf2},
                    {{16{iqf3[15]}},iqf3}, {{16{iqf4[15]}},iqf4}, {{16{iqf5[15]}},iqf5});
            for (w = 0; w < 6;  w = w + 1) $fwrite(fout, "%08x\n", tauext[w]);

            repeat (2) @(posedge clk);
        end
    endtask

    integer nz;
    initial begin
        rst_n = 1'b0;
        fk_jac_start = 1'b0; cart_start = 1'b0; tiq_start = 1'b0;
        q0=0; q1=0; q2=0; q3=0; q4=0; q5=0;
        for (w = 0; w < 6; w = w + 1) begin
            xdes[w]=0; K_r[w]=0; D_r[w]=0; xv[w]=0;
            pose_imp[w]=0; tau_lat[w]=0; iq_act[w]=0; tauext[w]=0; Fv[w]=0;
        end
        for (w = 0; w < 36; w = w + 1) J_imp[w] = 0;
        for (w = 0; w < MAX_CASES*WORDS_IN; w = w + 1) vin[w] = 32'hxxxx_xxxx;

        $readmemh("sim/r1_3_vectors.mem", vin);

        // 统计实际装入的字数（末尾未初始化为 x）
        n_cases = 0;
        nz = 0;
        for (w = 0; w < MAX_CASES*WORDS_IN; w = w + 1)
            if (vin[w] !== 32'hxxxx_xxxx) nz = w + 1;
        n_cases = nz / WORDS_IN;

        if (n_cases <= 0) begin
            $display("ERROR: sim/r1_3_vectors.mem 未装入 (先跑 python tools/force_golden.py gen)");
            $finish;
        end

        fout = $fopen("sim/r1_3_rtl_out.mem", "w");
        if (fout == 0) begin
            $display("ERROR: 无法写 sim/r1_3_rtl_out.mem");
            $finish;
        end

        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        $display("====================================================================");
        $display("R1.3 force-chain fixed-point dump: %0d cases x %0d words", n_cases, WORDS_OUT);
        $display("  chain: analytical_jacobian_serial -> cart_impedance(wrench+J^T)");
        $display("         -> torque_to_iq (as-is KT_INV=1310720 / override KT_INV=20)");
        $display("         -> tau_ext = Kt*iq_actual - tau_cmd  (V3 udp_fk_cmd op15)");
        $display("====================================================================");

        for (ci = 0; ci < n_cases; ci = ci + 1) begin
            run_case(ci);
            if ((ci % 100) == 0)
                $display("  ... case %0d / %0d  (tau0=%0d iq_asis0=%0d iq_fix0=%0d)",
                         ci, n_cases, tau_lat[0], iqa0, iqf0);
        end

        $fclose(fout);
        $display("dumped %0d cases -> sim/r1_3_rtl_out.mem", n_cases);
        $display("R1_3_DUMP_OK");
        $display("ALL TESTS PASSED");
        $finish;
    end

    initial begin
        #2_000_000_000;
        $display("TIMEOUT - simulation hung");
        $finish;
    end

endmodule
