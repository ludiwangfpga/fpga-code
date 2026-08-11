module eth_ctrl (
    input         clk                  ,
    input         rst                  ,

    input  [47:0] s_mac_src_mac        ,
    input  [47:0] s_mac_dst_mac        ,
    input  [15:0] s_mac_op_len         ,     
    input  [ 7:0] s_mac_axis_data      ,
    input         s_mac_axis_valid     ,
    input         s_mac_axis_last      ,
    input         s_mac_axis_error     , 

    output [47:0] m_arp_mac_src_mac    ,
    output [47:0] m_arp_mac_dst_mac    ,
    output [15:0] m_arp_mac_op_len     ,     
    output [ 7:0] m_arp_mac_axis_data  ,
    output        m_arp_mac_axis_valid ,
    output        m_arp_mac_axis_last  ,
    output        m_arp_mac_axis_error ,

    output [47:0] m_ip_mac_src_mac     ,
    output [47:0] m_ip_mac_dst_mac     ,
    output [15:0] m_ip_mac_op_len      ,     
    output [ 7:0] m_ip_mac_axis_data   ,
    output        m_ip_mac_axis_valid  ,
    output        m_ip_mac_axis_last   ,
    output        m_ip_mac_axis_error  ,

    input  [47:0] s_arp_mac_src_mac    ,
    input  [47:0] s_arp_mac_dst_mac    ,
    input  [15:0] s_arp_mac_op_len     ,     
    input  [ 7:0] s_arp_mac_axis_data  ,
    input         s_arp_mac_axis_valid ,
    output        s_arp_mac_axis_ready ,
    input         s_arp_mac_axis_last  ,
    output        s_arp_mac_axis_grant ,
    input         s_arp_mac_axis_req   ,

    input  [47:0] s_ip_mac_src_mac     ,
    input  [47:0] s_ip_mac_dst_mac     ,
    input  [15:0] s_ip_mac_op_len      ,     
    input  [ 7:0] s_ip_mac_axis_data   ,
    input         s_ip_mac_axis_valid  ,
    output        s_ip_mac_axis_ready  ,
    input         s_ip_mac_axis_last   ,
    output        s_ip_mac_axis_grant  ,
    input         s_ip_mac_axis_req    ,

    output [47:0] m_mac_src_mac        ,
    output [47:0] m_mac_dst_mac        ,
    output [15:0] m_mac_op_len         ,     
    output [ 7:0] m_mac_axis_data      ,
    output        m_mac_axis_valid     ,
    input         m_mac_axis_ready     ,
    output        m_mac_axis_last      
);
    
eth_rx_ctrl eth_rx_ctrl_inst(
    .clk                  (clk                 ),
    .rst                  (rst                 ),
    .s_mac_src_mac        (s_mac_src_mac       ),
    .s_mac_dst_mac        (s_mac_dst_mac       ),
    .s_mac_op_len         (s_mac_op_len        ),     
    .s_mac_axis_data      (s_mac_axis_data     ),
    .s_mac_axis_valid     (s_mac_axis_valid    ),
    .s_mac_axis_last      (s_mac_axis_last     ),
    .s_mac_axis_error     (s_mac_axis_error    ), 

    .m_arp_mac_src_mac    (m_arp_mac_src_mac   ),
    .m_arp_mac_dst_mac    (m_arp_mac_dst_mac   ),
    .m_arp_mac_op_len     (m_arp_mac_op_len    ),     
    .m_arp_mac_axis_data  (m_arp_mac_axis_data ),
    .m_arp_mac_axis_valid (m_arp_mac_axis_valid),
    .m_arp_mac_axis_last  (m_arp_mac_axis_last ),
    .m_arp_mac_axis_error (m_arp_mac_axis_error),

    .m_ip_mac_src_mac     (m_ip_mac_src_mac    ),
    .m_ip_mac_dst_mac     (m_ip_mac_dst_mac    ),
    .m_ip_mac_op_len      (m_ip_mac_op_len     ),     
    .m_ip_mac_axis_data   (m_ip_mac_axis_data  ),
    .m_ip_mac_axis_valid  (m_ip_mac_axis_valid ),
    .m_ip_mac_axis_last   (m_ip_mac_axis_last  ),
    .m_ip_mac_axis_error  (m_ip_mac_axis_error )
);


eth_tx_ctrl eth_tx_ctrl_inst(
    .clk                  (clk                 ),
    .rst                  (rst                 ),
    .s_arp_mac_src_mac    (s_arp_mac_src_mac   ),
    .s_arp_mac_dst_mac    (s_arp_mac_dst_mac   ),
    .s_arp_mac_op_len     (s_arp_mac_op_len    ),     
    .s_arp_mac_axis_data  (s_arp_mac_axis_data ),
    .s_arp_mac_axis_valid (s_arp_mac_axis_valid),
    .s_arp_mac_axis_ready (s_arp_mac_axis_ready),
    .s_arp_mac_axis_last  (s_arp_mac_axis_last ),
    .s_arp_mac_axis_grant (s_arp_mac_axis_grant),
    .s_arp_mac_axis_req   (s_arp_mac_axis_req  ),

    .s_ip_mac_src_mac     (s_ip_mac_src_mac    ),
    .s_ip_mac_dst_mac     (s_ip_mac_dst_mac    ),
    .s_ip_mac_op_len      (s_ip_mac_op_len     ),     
    .s_ip_mac_axis_data   (s_ip_mac_axis_data  ),
    .s_ip_mac_axis_valid  (s_ip_mac_axis_valid ),
    .s_ip_mac_axis_ready  (s_ip_mac_axis_ready ),
    .s_ip_mac_axis_last   (s_ip_mac_axis_last  ),
    .s_ip_mac_axis_grant  (s_ip_mac_axis_grant ),
    .s_ip_mac_axis_req    (s_ip_mac_axis_req   ),

    .m_mac_src_mac        (m_mac_src_mac       ),
    .m_mac_dst_mac        (m_mac_dst_mac       ),
    .m_mac_op_len         (m_mac_op_len        ),     
    .m_mac_axis_data      (m_mac_axis_data     ),
    .m_mac_axis_valid     (m_mac_axis_valid    ),
    .m_mac_axis_ready     (m_mac_axis_ready    ),
    .m_mac_axis_last      (m_mac_axis_last     ) 
);

endmodule