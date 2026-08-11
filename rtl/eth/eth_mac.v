module eth_mac(
    input         clk              ,
    input         rst              ,

    input [47:0]  cfg_mac          ,

    input  [ 7:0] s_eth_axis_data  ,
    input         s_eth_axis_valid ,
    input         s_eth_axis_last  ,
    input         s_eth_axis_error ,
    output [47:0] m_mac_src_mac    ,
    output [47:0] m_mac_dst_mac    ,
    output [15:0] m_mac_op_len     ,     
    output [ 7:0] m_mac_axis_data  ,
    output        m_mac_axis_valid ,
    output        m_mac_axis_last  ,
    output        m_mac_axis_error ,

    input  [47:0] s_mac_src_mac    ,
    input  [47:0] s_mac_dst_mac    ,
    input  [15:0] s_mac_op_len     ,     
    input  [ 7:0] s_mac_axis_data  ,
    input         s_mac_axis_valid ,
    output        s_mac_axis_ready ,
    input         s_mac_axis_last  ,

    output [ 7:0] m_eth_axis_data  ,
    output        m_eth_axis_valid ,
    input         m_eth_axis_ready ,
    output        m_eth_axis_last   
);

eth_mac_rx eth_mac_rx_inst(
    .clk              (clk              ),
    .rst              (rst              ),

    .cfg_mac          (cfg_mac          ),

    .s_eth_axis_data  (s_eth_axis_data  ),
    .s_eth_axis_valid (s_eth_axis_valid ),
    .s_eth_axis_last  (s_eth_axis_last  ),
    .s_eth_axis_error (s_eth_axis_error ),
    .m_mac_src_mac    (m_mac_src_mac    ),
    .m_mac_dst_mac    (m_mac_dst_mac    ),
    .m_mac_op_len     (m_mac_op_len     ),     
    .m_mac_axis_data  (m_mac_axis_data  ),
    .m_mac_axis_valid (m_mac_axis_valid ),
    .m_mac_axis_last  (m_mac_axis_last  ),
    .m_mac_axis_error (m_mac_axis_error ) 
);
    
eth_mac_tx_ctrl eth_mac_tx_ctrl_inst(
    .clk              (clk              ),
    .rst              (rst              ),
    
    .s_mac_src_mac    (s_mac_src_mac    ),
    .s_mac_dst_mac    (s_mac_dst_mac    ),
    .s_mac_op_len     (s_mac_op_len     ),     
    .s_mac_axis_data  (s_mac_axis_data  ),
    .s_mac_axis_valid (s_mac_axis_valid ),
    .s_mac_axis_ready (s_mac_axis_ready ),
    .s_mac_axis_last  (s_mac_axis_last  ),

    .m_eth_axis_data  (m_eth_axis_data  ),
    .m_eth_axis_valid (m_eth_axis_valid ),
    .m_eth_axis_ready (m_eth_axis_ready ),
    .m_eth_axis_last  (m_eth_axis_last  ) 
);
endmodule