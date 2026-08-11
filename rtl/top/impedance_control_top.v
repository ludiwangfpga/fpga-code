// impedance_control_top.v — M5.3-udp FOC 全 UDP 集成综合顶层
// 顶层端口只有以太网/MAC（与 top_udp_add 完全相同），无任何 FOC 物理引脚。
// phi/adc/duty 全走 udp_fk_cmd 内部寄存器；pwm/iq_actual 本阶段悬空。
// 网络参数写死：FPGA 172.28.109.173:8081 → host 172.28.109.174:8080, src MAC 11:22:33:44:55:66
// foc_clk = eth_clk (125MHz); rst_n = ~eth_rst。
// en_adc 适配器：sn_adc 延 1 拍当 en_adc，adc 用 udp_fk_cmd 寄存器值。

module impedance_control_top (
    // ---- PHY RGMII 接口（与 top_udp_add 完全相同）----
    input         phy_rgmii_rx_clk,
    input         phy_rgmii_rx_ctl,
    input   [3:0] phy_rgmii_rx_data,
    output        phy_rgmii_tx_clk,
    output        phy_rgmii_tx_ctl,
    output  [3:0] phy_rgmii_tx_data,
    input         pl_clk
    // ---- 无任何 FOC 物理端口 ----
);

    // ---- localparam（照搬 top_udp_add）----
    localparam CLOCK_BUFF = "BUFG";
    localparam DEVICE     = "XILINX";
    localparam IDDR_PRIM  = "IODDR";
    localparam CLK_90     = "FALSE";
    localparam FLUSH_TIME = 32'hFFFF_FFFF;
    localparam USE_FLUSH  = "FALSE";
    localparam RERNW_TIME = 16'hFFFF;

    // ---- 时钟 / 复位 ----
    wire eth_clk;
    wire eth_clk_90;
    wire clk_200M;
    wire npu_clk_unused;
    wire locked;
    wire eth_rst = ~locked;   // 高有效

    clk_wiz_0 clk_wiz_0_inst (
        .clk_out1(npu_clk_unused),
        .clk_out2(eth_clk),
        .clk_out3(eth_clk_90),
        .clk_out4(clk_200M),
        .locked  (locked),
        .clk_in1 (pl_clk)
    );

    // ---- 网络参数（写死，同 top_udp_add）----
    wire [15:0] cfg_udp_src_port = 16'd8081;
    wire [15:0] cfg_udp_dst_port = 16'd8080;
    wire [31:0] cfg_src_ip       = {8'd172, 8'd28, 8'd109, 8'd173}; // FPGA
    wire [31:0] cfg_dst_ip       = {8'd172, 8'd28, 8'd109, 8'd174}; // host
    wire [47:0] cfg_src_mac      = 48'h11_22_33_44_55_66;
    wire [10:0] cfg_udp_len;

    // ---- UDP AXIS 总线（eth_top ↔ udp_fk_cmd）----
    wire [7:0] rx_axis_data;
    wire       rx_axis_valid;
    wire       rx_axis_last;
    wire       rx_axis_error;

    wire [7:0] tx_axis_data;
    wire       tx_axis_valid;
    wire       tx_axis_last;
    wire       tx_axis_ready;

    // ---- 内部连接线 ----
    // iq: udp_fk_cmd.iq_out → foc_array.iq_ref
    wire signed [15:0] iqw0, iqw1, iqw2, iqw3, iqw4, iqw5;

    // duty: foc_array.duty → udp_fk_cmd.duty_*
    wire [9:0] da0, db0, dc0;
    wire [9:0] da1, db1, dc1;
    wire [9:0] da2, db2, dc2;
    wire [9:0] da3, db3, dc3;
    wire [9:0] da4, db4, dc4;
    wire [9:0] da5, db5, dc5;

    // phi: udp_fk_cmd.phi_* → foc_array.phi_*
    wire [11:0] ph0, ph1, ph2, ph3, ph4, ph5;

    // adc: udp_fk_cmd.adc_* → foc_array.adc_*
    wire [11:0] aa0, ab0, ac0;
    wire [11:0] aa1, ab1, ac1;
    wire [11:0] aa2, ab2, ac2;
    wire [11:0] aa3, ab3, ac3;
    wire [11:0] aa4, ab4, ac4;
    wire [11:0] aa5, ab5, ac5;

    // sn_adc: foc_array.sn_adc_* 内部 wire
    wire sn0, sn1, sn2, sn3, sn4, sn5;

    // en_adc 适配器：sn_adc 延 1 拍当 en_adc
    reg ead0, ead1, ead2, ead3, ead4, ead5;
    always @(posedge eth_clk) begin
        ead0 <= sn0; ead1 <= sn1; ead2 <= sn2;
        ead3 <= sn3; ead4 <= sn4; ead5 <= sn5;
    end

    wire foc_init_done;

    // ---- eth_top 例化 ----
    eth_top #(
        .CLOCK_BUFF(CLOCK_BUFF),
        .DEVICE    (DEVICE    ),
        .IDDR_PRIM (IDDR_PRIM ),
        .CLK_90    (CLK_90    ),
        .FLUSH_TIME(FLUSH_TIME),
        .USE_FLUSH (USE_FLUSH ),
        .RERNW_TIME(RERNW_TIME)
    ) eth_top_inst (
        .clk              (eth_clk          ),
        .rst              (eth_rst          ),
        .clk90            (eth_clk_90       ),
        .clk_200M         (clk_200M         ),
        .phy_rgmii_rx_clk (phy_rgmii_rx_clk ),
        .phy_rgmii_rx_ctl (phy_rgmii_rx_ctl ),
        .phy_rgmii_rx_data(phy_rgmii_rx_data),
        .phy_rgmii_tx_clk (phy_rgmii_tx_clk ),
        .phy_rgmii_tx_ctl (phy_rgmii_tx_ctl ),
        .phy_rgmii_tx_data(phy_rgmii_tx_data),
        .cfg_udp_src_port (cfg_udp_src_port ),
        .cfg_udp_dst_port (cfg_udp_dst_port ),
        .cfg_src_ip       (cfg_src_ip       ),
        .cfg_dst_ip       (cfg_dst_ip       ),
        .cfg_udp_len      (cfg_udp_len      ),
        .cfg_src_mac      (cfg_src_mac      ),
        .m_udp_axis_data  (rx_axis_data     ),
        .m_udp_axis_valid (rx_axis_valid    ),
        .m_udp_axis_last  (rx_axis_last     ),
        .m_udp_axis_error (rx_axis_error    ),
        .s_udp_axis_data  (tx_axis_data     ),
        .s_udp_axis_valid (tx_axis_valid    ),
        .s_udp_axis_last  (tx_axis_last     ),
        .s_udp_axis_ready (tx_axis_ready    )
    );

    // 自适应 τ_ext 用: foc_array.iq_actual → udp_fk_cmd.iq_act
    wire signed [15:0] iqaw0, iqaw1, iqaw2, iqaw3, iqaw4, iqaw5;

    // ---- udp_fk_cmd 例化（含 iq_out / duty_* / phi_* / adc_* / iq_act 端口）----
    udp_fk_cmd udp_fk_cmd_inst (
        .clk             (eth_clk      ),
        .rst             (eth_rst      ),
        .s_udp_axis_data (rx_axis_data ),
        .s_udp_axis_valid(rx_axis_valid),
        .s_udp_axis_last (rx_axis_last ),
        .s_udp_axis_error(rx_axis_error),
        .m_udp_axis_data (tx_axis_data ),
        .m_udp_axis_valid(tx_axis_valid),
        .m_udp_axis_last (tx_axis_last ),
        .m_udp_axis_ready(tx_axis_ready),
        .cfg_udp_len     (cfg_udp_len  ),
        // iq 输出 → foc_array.iq_ref
        .iq_out0(iqw0), .iq_out1(iqw1), .iq_out2(iqw2),
        .iq_out3(iqw3), .iq_out4(iqw4), .iq_out5(iqw5),
        // duty 输入 ← foc_array.duty
        .duty_a_0(da0), .duty_b_0(db0), .duty_c_0(dc0),
        .duty_a_1(da1), .duty_b_1(db1), .duty_c_1(dc1),
        .duty_a_2(da2), .duty_b_2(db2), .duty_c_2(dc2),
        .duty_a_3(da3), .duty_b_3(db3), .duty_c_3(dc3),
        .duty_a_4(da4), .duty_b_4(db4), .duty_c_4(dc4),
        .duty_a_5(da5), .duty_b_5(db5), .duty_c_5(dc5),
        // phi 输出 → foc_array.phi_*
        .phi_0(ph0), .phi_1(ph1), .phi_2(ph2),
        .phi_3(ph3), .phi_4(ph4), .phi_5(ph5),
        // adc 输出 → foc_array.adc_*
        .adc_a_0(aa0), .adc_b_0(ab0), .adc_c_0(ac0),
        .adc_a_1(aa1), .adc_b_1(ab1), .adc_c_1(ac1),
        .adc_a_2(aa2), .adc_b_2(ab2), .adc_c_2(ac2),
        .adc_a_3(aa3), .adc_b_3(ab3), .adc_c_3(ac3),
        .adc_a_4(aa4), .adc_b_4(ab4), .adc_c_4(ac4),
        .adc_a_5(aa5), .adc_b_5(ab5), .adc_c_5(ac5),
        // iq_actual ← foc_array (片上算 τ_ext = Kt·iq_actual − τ_cmd)
        .iq_act_0(iqaw0), .iq_act_1(iqaw1), .iq_act_2(iqaw2),
        .iq_act_3(iqaw3), .iq_act_4(iqaw4), .iq_act_5(iqaw5)
    );

    // ---- foc_array 例化 ----
    // Kp/Ki 为 sim 调出值，真机再标定；foc_clk = eth_clk；rst_n = ~eth_rst
    foc_array foc_inst (
        .foc_clk(eth_clk),
        .rst_n  (~eth_rst),
        .Kp(31'd15000),
        .Ki(31'd8000),

        // iq_ref ← udp_fk_cmd.iq_out
        .iq_ref_0(iqw0), .iq_ref_1(iqw1), .iq_ref_2(iqw2),
        .iq_ref_3(iqw3), .iq_ref_4(iqw4), .iq_ref_5(iqw5),

        // 编码器 ← udp_fk_cmd.phi_*（UDP 寄存器值）
        .phi_0(ph0), .phi_1(ph1), .phi_2(ph2),
        .phi_3(ph3), .phi_4(ph4), .phi_5(ph5),

        // ADC 接口（sn_adc 内部 wire，en_adc = sn_adc 延 1 拍，adc 用 udp 寄存器值）
        .sn_adc_0(sn0), .en_adc_0(ead0),
        .adc_a_0(aa0), .adc_b_0(ab0), .adc_c_0(ac0),
        .sn_adc_1(sn1), .en_adc_1(ead1),
        .adc_a_1(aa1), .adc_b_1(ab1), .adc_c_1(ac1),
        .sn_adc_2(sn2), .en_adc_2(ead2),
        .adc_a_2(aa2), .adc_b_2(ab2), .adc_c_2(ac2),
        .sn_adc_3(sn3), .en_adc_3(ead3),
        .adc_a_3(aa3), .adc_b_3(ab3), .adc_c_3(ac3),
        .sn_adc_4(sn4), .en_adc_4(ead4),
        .adc_a_4(aa4), .adc_b_4(ab4), .adc_c_4(ac4),
        .sn_adc_5(sn5), .en_adc_5(ead5),
        .adc_a_5(aa5), .adc_b_5(ab5), .adc_c_5(ac5),

        // PWM 输出：本阶段不出顶层（全走 UDP，只读占空比），悬空
        .pwm_en_0(), .pwm_a_0(), .pwm_b_0(), .pwm_c_0(),
        .pwm_en_1(), .pwm_a_1(), .pwm_b_1(), .pwm_c_1(),
        .pwm_en_2(), .pwm_a_2(), .pwm_b_2(), .pwm_c_2(),
        .pwm_en_3(), .pwm_a_3(), .pwm_b_3(), .pwm_c_3(),
        .pwm_en_4(), .pwm_a_4(), .pwm_b_4(), .pwm_c_4(),
        .pwm_en_5(), .pwm_a_5(), .pwm_b_5(), .pwm_c_5(),

        // iq_actual → udp_fk_cmd (片上 τ_ext = Kt·iq_actual − τ_cmd, 闭环自适应)
        .iq_actual_0(iqaw0), .iq_actual_1(iqaw1), .iq_actual_2(iqaw2),
        .iq_actual_3(iqaw3), .iq_actual_4(iqaw4), .iq_actual_5(iqaw5),

        // all_init_done 内部 wire（不出顶层）
        .all_init_done(foc_init_done),

        // duty 输出 → udp_fk_cmd.duty_*
        .duty_a_0(da0), .duty_b_0(db0), .duty_c_0(dc0),
        .duty_a_1(da1), .duty_b_1(db1), .duty_c_1(dc1),
        .duty_a_2(da2), .duty_b_2(db2), .duty_c_2(dc2),
        .duty_a_3(da3), .duty_b_3(db3), .duty_c_3(dc3),
        .duty_a_4(da4), .duty_b_4(db4), .duty_c_4(dc4),
        .duty_a_5(da5), .duty_b_5(db5), .duty_c_5(dc5)
    );

endmodule
