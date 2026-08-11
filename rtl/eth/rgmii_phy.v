module rgmii_phy #(
    parameter CLOCK_BUFF = "BUFG"  ,
    parameter DEVICE     = "XILINX",
    parameter IDDR_PRIM  = "IODDR" ,
    parameter CLK_90     = "TRUE"   
) (
    input        clk              ,
    input        rst              ,
    input        clk90            ,
    input        clk_200M         ,

    input        phy_rgmii_rx_clk ,
    input        phy_rgmii_rx_ctl ,
    input  [3:0] phy_rgmii_rx_data,
    output       phy_rgmii_tx_clk ,
    output       phy_rgmii_tx_ctl ,
    output [3:0] phy_rgmii_tx_data,

    output [7:0] m_mac_gmii_data  ,
    output       m_mac_gmii_clk   ,
    output       m_mac_gmii_dv    ,
    output       m_mac_gmii_err   ,
    input  [7:0] s_mac_gmii_data  ,
    output       s_mac_gmii_clk   ,
    input        s_mac_gmii_dv     

);

rgmii_rx #(
    .CLOCK_BUFF  (CLOCK_BUFF),
    .DEVICE      (DEVICE    ),
    .IDDR_PRIM   (IDDR_PRIM )
) rgmii_rx_inst (
    .clk_200M       (clk_200M         ),
    .phy_rgmii_clk  (phy_rgmii_rx_clk ),
    .phy_rgmii_ctl  (phy_rgmii_rx_ctl ),
    .phy_rgmii_data (phy_rgmii_rx_data),

    .mac_gmii_data  (m_mac_gmii_data  ),
    .mac_gmii_clk   (m_mac_gmii_clk   ),
    .mac_gmii_dv    (m_mac_gmii_dv    ),
    .mac_gmii_err   (m_mac_gmii_err   ) 

);

rgmii_tx #(
    .DEVICE      (DEVICE   ),
    .IDDR_PRIM   (IDDR_PRIM),
    .CLK_90      (CLK_90   )
)rgmii_tx_inst (
    .clk             (clk              ),
    .clk_90          (clk90            ),

    .phy_rgmii_clk   (phy_rgmii_tx_clk ),
    .phy_rgmii_ctl   (phy_rgmii_tx_ctl ),
    .phy_rgmii_data  (phy_rgmii_tx_data),

    .mac_gmii_data   (s_mac_gmii_data  ),
    .mac_gmii_clk    (s_mac_gmii_clk   ),
    .mac_gmii_dv     (s_mac_gmii_dv    ) 
);
    
endmodule