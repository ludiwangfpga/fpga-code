// 综合顶层：clk_wiz_0 + eth_top + udp_fk_cmd。单一 eth_clk 域。
// 网络参数沿用原 yolo_7100/src/top.v；eth_rst = ~locked（高有效）。
module top_udp_add (
    input         phy_rgmii_rx_clk,
    input         phy_rgmii_rx_ctl,
    input   [3:0] phy_rgmii_rx_data,
    output        phy_rgmii_tx_clk,
    output        phy_rgmii_tx_ctl,
    output  [3:0] phy_rgmii_tx_data,
    input         pl_clk
);
    localparam CLOCK_BUFF = "BUFG";
    localparam DEVICE     = "XILINX";
    localparam IDDR_PRIM  = "IODDR";
    localparam CLK_90     = "FALSE";
    localparam FLUSH_TIME = 32'hFFFF_FFFF;
    localparam USE_FLUSH  = "FALSE";
    localparam RERNW_TIME = 16'hFFFF;

    wire eth_clk;
    wire eth_clk_90;
    wire clk_200M;
    wire npu_clk_unused;       // clk_wiz clk_out1 未用（沿用原 IP 配置）
    wire locked;
    // locked 是 MMCM 干净的寄存输出，复位释放无毛刺；demo 不需额外复位同步器
    wire eth_rst = ~locked;

    clk_wiz_0 clk_wiz_0_inst (
        .clk_out1(npu_clk_unused),
        .clk_out2(eth_clk),
        .clk_out3(eth_clk_90),
        .clk_out4(clk_200M),
        .locked  (locked),
        .clk_in1 (pl_clk)
    );

    wire [15:0] cfg_udp_src_port = 16'd8081;
    wire [15:0] cfg_udp_dst_port = 16'd8080;
    wire [31:0] cfg_src_ip       = {8'd172, 8'd28, 8'd109, 8'd173}; // FPGA 172.28.109.173
    wire [31:0] cfg_dst_ip       = {8'd172, 8'd28, 8'd109, 8'd174}; // host 172.28.109.174
    wire [47:0] cfg_src_mac      = 48'h11_22_33_44_55_66;
    wire [10:0] cfg_udp_len;

    wire [7:0] rx_axis_data;   // eth_top RX -> udp_fk_cmd
    wire       rx_axis_valid;
    wire       rx_axis_last;
    wire       rx_axis_error;

    wire [7:0] tx_axis_data;   // udp_fk_cmd -> eth_top TX
    wire       tx_axis_valid;
    wire       tx_axis_last;
    wire       tx_axis_ready;

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
        .cfg_udp_len     (cfg_udp_len  )
    );

endmodule
