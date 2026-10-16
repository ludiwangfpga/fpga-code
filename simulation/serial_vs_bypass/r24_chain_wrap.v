`timescale 1ns / 1ps
// ============================================================================
// r24_chain_wrap.v -- R2.4 experiment B: thin wrapper that composes the
//   EXISTING deployed modules (no control-law rewrite) in two topologies.
//
//   SERIAL = 0  (BYPASS, mirrors the deployed udp_fk_cmd.v):
//       start -> [FK+J 422] -> [cart_impedance 119] -> [torque_to_iq 4] -> done
//       adaptation (adaptive_impedance_v2) is triggered in the same cycle as
//       FK when adapt_req=1 and runs OFF the critical path; its Ks/Ds outputs
//       are latched into ksout/dsout whenever adapt_done rises and reach the
//       impedance stage through the same adaptive_en mux as in udp_fk_cmd.v.
//
//   SERIAL = 1  (SERIAL reference: same law, same Q16.16 format, same I/O):
//       start -> [FK+J] -> [adaptive_impedance_v2 update, wait done] ->
//                latch Ks/Ds -> [cart_impedance] -> [torque_to_iq] -> done
//       i.e. the forward chain must wait for the adaptation update to finish
//       before the gains are applied.  When adapt_req=0 the update is skipped,
//       so the forward latency of the serial design depends on whether an
//       update is scheduled on this beat (this is the jitter that decoupling
//       removes).
//
//   Handshake style is identical to udp_fk_cmd.v: a *_done observed in cycle
//   t makes the next *_start high in cycle t+1 (0 idle cycles in between).
// ============================================================================
module r24_chain_wrap #(
    parameter SERIAL = 0
) (
    input  wire clk, rst_n, start, adapt_req, adaptive_en,
    input  wire signed [31:0] q0,q1,q2,q3,q4,q5,
    input  wire signed [31:0] xd0,xd1,xd2,xd3,xd4,xd5,
    input  wire signed [31:0] xv0,xv1,xv2,xv3,xv4,xv5,
    input  wire signed [31:0] Kin0,Kin1,Kin2,Kin3,Kin4,Kin5,
    input  wire signed [31:0] Din0,Din1,Din2,Din3,Din4,Din5,
    input  wire signed [31:0] qe0,qe1,qe2,qe3,qe4,qe5,
    input  wire signed [31:0] te0,te1,te2,te3,te4,te5,
    input  wire signed [31:0] ksn0,ksn1,ksn2,ksn3,ksn4,ksn5,
    input  wire signed [31:0] dsn0,dsn1,dsn2,dsn3,dsn4,dsn5,
    output reg  signed [15:0] iq0,iq1,iq2,iq3,iq4,iq5,
    output wire signed [31:0] Ks0,Ks1,Ks2,Ks3,Ks4,Ks5,
    output reg  done
);
    localparam S_IDLE=3'd0, S_FK=3'd1, S_ADAPT=3'd2, S_IMP=3'd3, S_TIQ=3'd4;
    reg [2:0] state;
    reg fk_start, adapt_start, imp_start, tiq_start;
    wire fk_done, adapt_done, imp_done, tiq_done;

    // ---- FK + analytical Jacobian (deployed module) ----
    wire signed [31:0] fk_x, fk_y, fk_z, fk_rx, fk_ry, fk_rz;
    wire signed [31:0] j_w [0:35];
    reg  signed [31:0] j_reg [0:35];
    reg  signed [31:0] pose_reg [0:5];
    analytical_jacobian_serial jac_inst (
        .clk(clk), .rst_n(rst_n), .start(fk_start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .pose_x(fk_x), .pose_y(fk_y), .pose_z(fk_z), .pose_rx(fk_rx), .pose_ry(fk_ry), .pose_rz(fk_rz),
        .J00(j_w[0]),  .J01(j_w[1]),  .J02(j_w[2]),  .J03(j_w[3]),  .J04(j_w[4]),  .J05(j_w[5]),
        .J10(j_w[6]),  .J11(j_w[7]),  .J12(j_w[8]),  .J13(j_w[9]),  .J14(j_w[10]), .J15(j_w[11]),
        .J20(j_w[12]), .J21(j_w[13]), .J22(j_w[14]), .J23(j_w[15]), .J24(j_w[16]), .J25(j_w[17]),
        .J30(j_w[18]), .J31(j_w[19]), .J32(j_w[20]), .J33(j_w[21]), .J34(j_w[22]), .J35(j_w[23]),
        .J40(j_w[24]), .J41(j_w[25]), .J42(j_w[26]), .J43(j_w[27]), .J44(j_w[28]), .J45(j_w[29]),
        .J50(j_w[30]), .J51(j_w[31]), .J52(j_w[32]), .J53(j_w[33]), .J54(j_w[34]), .J55(j_w[35]),
        .done(fk_done));

    // ---- adaptation (deployed module) ----
    wire signed [31:0] aKs [0:5], aDs [0:5], aKe [0:5];
    reg  signed [31:0] ksout [0:5], dsout [0:5];
    adaptive_impedance_v2 adapt_inst (
        .clk(clk), .rst_n(rst_n), .start(adapt_start),
        .tau_ext_0(te0), .tau_ext_1(te1), .tau_ext_2(te2), .tau_ext_3(te3), .tau_ext_4(te4), .tau_ext_5(te5),
        .q_err_0(qe0), .q_err_1(qe1), .q_err_2(qe2), .q_err_3(qe3), .q_err_4(qe4), .q_err_5(qe5),
        .Ks_nom_0(ksn0), .Ks_nom_1(ksn1), .Ks_nom_2(ksn2), .Ks_nom_3(ksn3), .Ks_nom_4(ksn4), .Ks_nom_5(ksn5),
        .Ds_nom_0(dsn0), .Ds_nom_1(dsn1), .Ds_nom_2(dsn2), .Ds_nom_3(dsn3), .Ds_nom_4(dsn4), .Ds_nom_5(dsn5),
        .adaptive_enable(adaptive_en), .prior_load(1'b0),
        .K_env_prior_0(32'sd0), .K_env_prior_1(32'sd0), .K_env_prior_2(32'sd0), .K_env_prior_3(32'sd0), .K_env_prior_4(32'sd0), .K_env_prior_5(32'sd0),
        .P_prior_0(32'sd0), .P_prior_1(32'sd0), .P_prior_2(32'sd0), .P_prior_3(32'sd0), .P_prior_4(32'sd0), .P_prior_5(32'sd0),
        .Ks_out_0(aKs[0]), .Ks_out_1(aKs[1]), .Ks_out_2(aKs[2]), .Ks_out_3(aKs[3]), .Ks_out_4(aKs[4]), .Ks_out_5(aKs[5]),
        .Ds_out_0(aDs[0]), .Ds_out_1(aDs[1]), .Ds_out_2(aDs[2]), .Ds_out_3(aDs[3]), .Ds_out_4(aDs[4]), .Ds_out_5(aDs[5]),
        .K_env_est_0(aKe[0]), .K_env_est_1(aKe[1]), .K_env_est_2(aKe[2]), .K_env_est_3(aKe[3]), .K_env_est_4(aKe[4]), .K_env_est_5(aKe[5]),
        .done(adapt_done));
    assign Ks0=ksout[0]; assign Ks1=ksout[1]; assign Ks2=ksout[2];
    assign Ks3=ksout[3]; assign Ks4=ksout[4]; assign Ks5=ksout[5];

    // ---- gain mux (identical to udp_fk_cmd.v g_gainmux) ----
    wire signed [31:0] Kc [0:5], Dc [0:5];
    wire signed [31:0] Kin [0:5], Din [0:5];
    assign Kin[0]=Kin0; assign Kin[1]=Kin1; assign Kin[2]=Kin2; assign Kin[3]=Kin3; assign Kin[4]=Kin4; assign Kin[5]=Kin5;
    assign Din[0]=Din0; assign Din[1]=Din1; assign Din[2]=Din2; assign Din[3]=Din3; assign Din[4]=Din4; assign Din[5]=Din5;
    genvar gk;
    generate for (gk=0; gk<6; gk=gk+1) begin : g_gainmux
        assign Kc[gk] = adaptive_en ? ksout[gk] : Kin[gk];
        assign Dc[gk] = adaptive_en ? dsout[gk] : Din[gk];
    end endgenerate

    // ---- Cartesian impedance + J^T (deployed module) ----
    wire signed [31:0] tau_w [0:5];
    reg  signed [31:0] tau_reg [0:5];
    cart_impedance imp_inst (
        .clk(clk), .rst_n(rst_n), .start(imp_start),
        .J00(j_reg[0]),  .J01(j_reg[1]),  .J02(j_reg[2]),  .J03(j_reg[3]),  .J04(j_reg[4]),  .J05(j_reg[5]),
        .J10(j_reg[6]),  .J11(j_reg[7]),  .J12(j_reg[8]),  .J13(j_reg[9]),  .J14(j_reg[10]), .J15(j_reg[11]),
        .J20(j_reg[12]), .J21(j_reg[13]), .J22(j_reg[14]), .J23(j_reg[15]), .J24(j_reg[16]), .J25(j_reg[17]),
        .J30(j_reg[18]), .J31(j_reg[19]), .J32(j_reg[20]), .J33(j_reg[21]), .J34(j_reg[22]), .J35(j_reg[23]),
        .J40(j_reg[24]), .J41(j_reg[25]), .J42(j_reg[26]), .J43(j_reg[27]), .J44(j_reg[28]), .J45(j_reg[29]),
        .J50(j_reg[30]), .J51(j_reg[31]), .J52(j_reg[32]), .J53(j_reg[33]), .J54(j_reg[34]), .J55(j_reg[35]),
        .p0(pose_reg[0]), .p1(pose_reg[1]), .p2(pose_reg[2]), .p3(pose_reg[3]), .p4(pose_reg[4]), .p5(pose_reg[5]),
        .xd0(xd0), .xd1(xd1), .xd2(xd2), .xd3(xd3), .xd4(xd4), .xd5(xd5),
        .K0(Kc[0]), .K1(Kc[1]), .K2(Kc[2]), .K3(Kc[3]), .K4(Kc[4]), .K5(Kc[5]),
        .D0(Dc[0]), .D1(Dc[1]), .D2(Dc[2]), .D3(Dc[3]), .D4(Dc[4]), .D5(Dc[5]),
        .xv0(xv0), .xv1(xv1), .xv2(xv2), .xv3(xv3), .xv4(xv4), .xv5(xv5),
        .tau0(tau_w[0]), .tau1(tau_w[1]), .tau2(tau_w[2]), .tau3(tau_w[3]), .tau4(tau_w[4]), .tau5(tau_w[5]),
        .done(imp_done));

    // ---- torque -> iq (deployed module) ----
    wire signed [15:0] iq_w0, iq_w1, iq_w2, iq_w3, iq_w4, iq_w5;
    torque_to_iq tiq_inst (
        .clk(clk), .rst_n(rst_n), .start(tiq_start),
        .tau0(tau_reg[0]), .tau1(tau_reg[1]), .tau2(tau_reg[2]), .tau3(tau_reg[3]), .tau4(tau_reg[4]), .tau5(tau_reg[5]),
        .iq0(iq_w0), .iq1(iq_w1), .iq2(iq_w2), .iq3(iq_w3), .iq4(iq_w4), .iq5(iq_w5),
        .done(tiq_done));

    integer k;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state<=S_IDLE; done<=0; fk_start<=0; adapt_start<=0; imp_start<=0; tiq_start<=0;
            for (k=0;k<36;k=k+1) j_reg[k]<=0;
            for (k=0;k<6;k=k+1) begin pose_reg[k]<=0; tau_reg[k]<=0; ksout[k]<=0; dsout[k]<=0; end
            iq0<=0; iq1<=0; iq2<=0; iq3<=0; iq4<=0; iq5<=0;
        end else begin
            fk_start<=0; adapt_start<=0; imp_start<=0; tiq_start<=0; done<=0;
            // bypass: latch adapted gains whenever the side branch finishes (as S_ADAPT_WAIT does)
            if (SERIAL==0 && adapt_done)
                for (k=0;k<6;k=k+1) begin ksout[k]<=aKs[k]; dsout[k]<=aDs[k]; end
            case (state)
            S_IDLE: if (start) begin
                        fk_start<=1'b1;
                        if (SERIAL==0) adapt_start<=adapt_req;     // off-path, concurrent
                        state<=S_FK;
                    end
            S_FK:   if (fk_done) begin
                        pose_reg[0]<=fk_x; pose_reg[1]<=fk_y; pose_reg[2]<=fk_z;
                        pose_reg[3]<=fk_rx; pose_reg[4]<=fk_ry; pose_reg[5]<=fk_rz;
                        for (k=0;k<36;k=k+1) j_reg[k]<=j_w[k];
                        if (SERIAL==1 && adapt_req) begin adapt_start<=1'b1; state<=S_ADAPT; end
                        else begin imp_start<=1'b1; state<=S_IMP; end
                    end
            S_ADAPT: if (adapt_done) begin                            // serial: gains applied only now
                        for (k=0;k<6;k=k+1) begin ksout[k]<=aKs[k]; dsout[k]<=aDs[k]; end
                        imp_start<=1'b1; state<=S_IMP;
                    end
            S_IMP:  if (imp_done) begin
                        for (k=0;k<6;k=k+1) tau_reg[k]<=tau_w[k];
                        tiq_start<=1'b1; state<=S_TIQ;
                    end
            S_TIQ:  if (tiq_done) begin
                        iq0<=iq_w0; iq1<=iq_w1; iq2<=iq_w2; iq3<=iq_w3; iq4<=iq_w4; iq5<=iq_w5;
                        done<=1'b1; state<=S_IDLE;
                    end
            default: state<=S_IDLE;
            endcase
        end
    end
endmodule
