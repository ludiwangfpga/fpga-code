# RGMII + pl_clk 引脚/IO 标准/接口时钟约束（摘自原 yolo_7100 top.xdc，去 DDR/SYS_CLK）。
# 与原工程一致，本文件不含 pl_clk 的 create_clock 及 RGMII set_input/output_delay：
# 这些属真实上板时序收敛约束，依赖板级时钟频率与 PHY 数据手册，超出本 a+b demo
# 范围；上板时按板卡补齐。clk_wiz_0 输入时钟频率由其 .xci 内置 OOC 约束提供。
set_property PACKAGE_PIN AD18 [get_ports pl_clk]
set_property IOSTANDARD LVCMOS33 [get_ports pl_clk]

create_clock -period 8.000 -name phy_rxc [get_ports phy_rgmii_rx_clk]
create_clock -period 8.000 -name phy_txc [get_ports phy_rgmii_tx_clk]

set_property IOSTANDARD LVCMOS18 [get_ports phy_rgmii_rx_ctl]
set_property IOSTANDARD LVCMOS18 [get_ports phy_rgmii_rx_clk]
set_property IOSTANDARD LVCMOS18 [get_ports phy_rgmii_tx_ctl]
set_property IOSTANDARD LVCMOS18 [get_ports phy_rgmii_tx_clk]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_rx_data[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_rx_data[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_rx_data[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_rx_data[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_tx_data[3]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_tx_data[2]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_tx_data[1]}]
set_property IOSTANDARD LVCMOS18 [get_ports {phy_rgmii_tx_data[0]}]

set_property PACKAGE_PIN AG20 [get_ports phy_rgmii_rx_ctl]
set_property PACKAGE_PIN AF20 [get_ports phy_rgmii_rx_clk]
set_property PACKAGE_PIN AF19 [get_ports phy_rgmii_tx_ctl]
set_property PACKAGE_PIN Y22  [get_ports phy_rgmii_tx_clk]
set_property PACKAGE_PIN AB21 [get_ports {phy_rgmii_tx_data[0]}]
set_property PACKAGE_PIN AA22 [get_ports {phy_rgmii_tx_data[1]}]
set_property PACKAGE_PIN AD21 [get_ports {phy_rgmii_tx_data[2]}]
set_property PACKAGE_PIN AB22 [get_ports {phy_rgmii_tx_data[3]}]
set_property PACKAGE_PIN AJ20 [get_ports {phy_rgmii_rx_data[0]}]
set_property PACKAGE_PIN AE21 [get_ports {phy_rgmii_rx_data[1]}]
set_property PACKAGE_PIN AG21 [get_ports {phy_rgmii_rx_data[2]}]
set_property PACKAGE_PIN AH21 [get_ports {phy_rgmii_rx_data[3]}]
