module eth_top #(
    parameter CLOCK_BUFF = "BUFG"       ,
    parameter DEVICE     = "XILINX"     ,
    parameter IDDR_PRIM  = "IODDR"      ,
    parameter CLK_90     = "TRUE"       ,
    parameter FLUSH_TIME = 32'hFFFF_FFFF,
    parameter USE_FLUSH  = "TRUE"       ,
    parameter RERNW_TIME = 16'hFFFF         
) (
    input         clk              ,
    input         rst              ,
    input         clk90            ,
    input         clk_200M         ,

    input         phy_rgmii_rx_clk ,
    input         phy_rgmii_rx_ctl ,
    input   [3:0] phy_rgmii_rx_data,
    output        phy_rgmii_tx_clk ,
    output        phy_rgmii_tx_ctl ,
    output  [3:0] phy_rgmii_tx_data,

    input  [15:0] cfg_udp_src_port ,
    input  [15:0] cfg_udp_dst_port , 
    input  [31:0] cfg_src_ip       ,
    input  [31:0] cfg_dst_ip       ,
    input  [10:0] cfg_udp_len      ,
    input  [47:0] cfg_src_mac      ,

    output [ 7:0] m_udp_axis_data  ,
    output        m_udp_axis_valid ,
    output        m_udp_axis_last  ,
    output        m_udp_axis_error ,

    input  [ 7:0] s_udp_axis_data  ,
    input         s_udp_axis_valid ,
    input         s_udp_axis_last  ,
    output        s_udp_axis_ready 
);

wire [7:0] m_mac_gmii_data;
wire       m_mac_gmii_clk ;
wire       m_mac_gmii_dv  ;
wire       m_mac_gmii_err ;
wire [7:0] s_mac_gmii_data;
wire       s_mac_gmii_clk ;
wire       s_mac_gmii_dv  ;

wire [7:0] m_eth_axis_data ;
wire       m_eth_axis_valid;
wire       m_eth_axis_last ;
wire       m_eth_axis_error;
wire [7:0] s_eth_axis_data ;
wire       s_eth_axis_valid;
wire       s_eth_axis_last ;
wire       s_eth_axis_ready;

wire [47:0] m_mac_src_mac   ; 
wire [47:0] m_mac_dst_mac   ; 
wire [15:0] m_mac_op_len    ; 
wire [ 7:0] m_mac_axis_data ; 
wire        m_mac_axis_valid; 
wire        m_mac_axis_last ; 
wire        m_mac_axis_error; 
wire [47:0] s_mac_src_mac   ;
wire [47:0] s_mac_dst_mac   ;
wire [15:0] s_mac_op_len    ;
wire [ 7:0] s_mac_axis_data ;
wire        s_mac_axis_valid;
wire        s_mac_axis_ready;
wire        s_mac_axis_last ;

wire [47:0] m_arp_mac_src_mac   ;
wire [47:0] m_arp_mac_dst_mac   ;
wire [15:0] m_arp_mac_op_len    ;
wire [ 7:0] m_arp_mac_axis_data ;
wire        m_arp_mac_axis_valid;
wire        m_arp_mac_axis_last ;
wire        m_arp_mac_axis_error;
wire [47:0] m_ip_mac_src_mac    ;
wire [47:0] m_ip_mac_dst_mac    ;
wire [15:0] m_ip_mac_op_len     ;
wire [ 7:0] m_ip_mac_axis_data  ;
wire        m_ip_mac_axis_valid ;
wire        m_ip_mac_axis_last  ;
wire        m_ip_mac_axis_error ;
wire [47:0] s_arp_mac_src_mac   ;
wire [47:0] s_arp_mac_dst_mac   ;
wire [15:0] s_arp_mac_op_len    ;
wire [ 7:0] s_arp_mac_axis_data ;
wire        s_arp_mac_axis_valid;
wire        s_arp_mac_axis_ready;
wire        s_arp_mac_axis_last ;
wire        s_arp_mac_axis_grant;
wire        s_arp_mac_axis_req  ;
wire [47:0] s_ip_mac_src_mac    ;
wire [47:0] s_ip_mac_dst_mac    ;
wire [15:0] s_ip_mac_op_len     ;
wire [ 7:0] s_ip_mac_axis_data  ;
wire        s_ip_mac_axis_valid ;
wire        s_ip_mac_axis_ready ;
wire        s_ip_mac_axis_last  ;
wire        s_ip_mac_axis_grant ;
wire        s_ip_mac_axis_req   ;

wire        query_ready         ;
wire        query_valid         ;
wire [31:0] query_ip            ;
wire [47:0] query_resp_mac      ;
wire [31:0] query_resp_ip       ;
wire        query_resp_valid    ;

rgmii_phy #(
    .CLOCK_BUFF  (CLOCK_BUFF ),
    .DEVICE      (DEVICE     ),
    .IDDR_PRIM   (IDDR_PRIM  ),
    .CLK_90      (CLK_90     ) 
) rgmii_phy_inst (
    .clk              (clk              ),
    .rst              (rst              ),
    .clk90            (clk90            ),
    .clk_200M         (clk_200M         ),

    .phy_rgmii_rx_clk (phy_rgmii_rx_clk ),
    .phy_rgmii_rx_ctl (phy_rgmii_rx_ctl ),
    .phy_rgmii_rx_data(phy_rgmii_rx_data),
    .phy_rgmii_tx_clk (phy_rgmii_tx_clk ),
    .phy_rgmii_tx_ctl (phy_rgmii_tx_ctl ),
    .phy_rgmii_tx_data(phy_rgmii_tx_data),

    .m_mac_gmii_data  (m_mac_gmii_data  ),
    .m_mac_gmii_clk   (m_mac_gmii_clk   ),
    .m_mac_gmii_dv    (m_mac_gmii_dv    ),
    .m_mac_gmii_err   (m_mac_gmii_err   ),
    .s_mac_gmii_data  (s_mac_gmii_data  ),
    .s_mac_gmii_clk   (s_mac_gmii_clk   ),
    .s_mac_gmii_dv    (s_mac_gmii_dv    ) 

);

gmii_axis#(
    .DEVICE    (DEVICE     )
) gmii_axis_inst (
    .clk              (clk              ),
    .rst              (rst              ),

    .mac_rx_gmii_clk  (m_mac_gmii_clk   ),
    .mac_rx_gmii_data (m_mac_gmii_data  ),
    .mac_rx_gmii_dv   (m_mac_gmii_dv    ),
    .mac_rx_gmii_err  (m_mac_gmii_err   ),    
    .m_eth_axis_data  (m_eth_axis_data  ),
    .m_eth_axis_valid (m_eth_axis_valid ),
    .m_eth_axis_last  (m_eth_axis_last  ),
    .m_eth_axis_error (m_eth_axis_error ),

    .mac_tx_gmii_clk  (s_mac_gmii_clk   ),
    .mac_tx_gmii_data (s_mac_gmii_data  ),
    .mac_tx_gmii_dv   (s_mac_gmii_dv    ),
    .s_eth_axis_data  (s_eth_axis_data  ),
    .s_eth_axis_valid (s_eth_axis_valid ),
    .s_eth_axis_last  (s_eth_axis_last  ),
    .s_eth_axis_ready (s_eth_axis_ready ) 
);

eth_mac eth_mac_inst(
    .clk              (clk              ),
    .rst              (rst              ),

    .cfg_mac          (cfg_src_mac      ),

    .s_eth_axis_data  (m_eth_axis_data  ),
    .s_eth_axis_valid (m_eth_axis_valid ),
    .s_eth_axis_last  (m_eth_axis_last  ),
    .s_eth_axis_error (m_eth_axis_error ),
    .m_mac_src_mac    (m_mac_src_mac    ),
    .m_mac_dst_mac    (m_mac_dst_mac    ),
    .m_mac_op_len     (m_mac_op_len     ),     
    .m_mac_axis_data  (m_mac_axis_data  ),
    .m_mac_axis_valid (m_mac_axis_valid ),
    .m_mac_axis_last  (m_mac_axis_last  ),
    .m_mac_axis_error (m_mac_axis_error ),

    .s_mac_src_mac    (s_mac_src_mac    ),
    .s_mac_dst_mac    (s_mac_dst_mac    ),
    .s_mac_op_len     (s_mac_op_len     ),     
    .s_mac_axis_data  (s_mac_axis_data  ),
    .s_mac_axis_valid (s_mac_axis_valid ),
    .s_mac_axis_ready (s_mac_axis_ready ),
    .s_mac_axis_last  (s_mac_axis_last  ),

    .m_eth_axis_data  (s_eth_axis_data  ),
    .m_eth_axis_valid (s_eth_axis_valid ),
    .m_eth_axis_ready (s_eth_axis_ready ),
    .m_eth_axis_last  (s_eth_axis_last  ) 
);

eth_ctrl eth_ctrl_inst(
    .clk                  (clk                 ),
    .rst                  (rst                 ),

    .s_mac_src_mac        (m_mac_src_mac       ),
    .s_mac_dst_mac        (m_mac_dst_mac       ),
    .s_mac_op_len         (m_mac_op_len        ),     
    .s_mac_axis_data      (m_mac_axis_data     ),
    .s_mac_axis_valid     (m_mac_axis_valid    ),
    .s_mac_axis_last      (m_mac_axis_last     ),
    .s_mac_axis_error     (m_mac_axis_error    ), 

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
    .m_ip_mac_axis_error  (m_ip_mac_axis_error ),

    .s_arp_mac_src_mac    (s_arp_mac_src_mac   ),
    .s_arp_mac_dst_mac    (s_arp_mac_dst_mac   ),
    .s_arp_mac_op_len     (s_arp_mac_op_len    ),     
    .s_arp_mac_axis_data  (s_arp_mac_axis_data ),
    .s_arp_mac_axis_valid (s_arp_mac_axis_valid),
    .s_arp_mac_axis_ready (s_arp_mac_axis_ready),
    .s_arp_mac_axis_last  (s_arp_mac_axis_last ),
    .s_arp_mac_axis_grant (s_arp_mac_axis_grant),
    .s_arp_mac_axis_req   (s_arp_mac_axis_req  ),

    .s_ip_mac_src_mac     (s_ip_mac_src_mac   ),
    .s_ip_mac_dst_mac     (s_ip_mac_dst_mac   ),
    .s_ip_mac_op_len      (s_ip_mac_op_len    ),     
    .s_ip_mac_axis_data   (s_ip_mac_axis_data ),
    .s_ip_mac_axis_valid  (s_ip_mac_axis_valid),
    .s_ip_mac_axis_ready  (s_ip_mac_axis_ready),
    .s_ip_mac_axis_last   (s_ip_mac_axis_last ),
    .s_ip_mac_axis_grant  (s_ip_mac_axis_grant),
    .s_ip_mac_axis_req    (s_ip_mac_axis_req  ),

    .m_mac_src_mac        (s_mac_src_mac      ),
    .m_mac_dst_mac        (s_mac_dst_mac      ),
    .m_mac_op_len         (s_mac_op_len       ),     
    .m_mac_axis_data      (s_mac_axis_data    ),
    .m_mac_axis_valid     (s_mac_axis_valid   ),
    .m_mac_axis_ready     (s_mac_axis_ready   ),
    .m_mac_axis_last      (s_mac_axis_last    )
);

arp #(
    .FLUSH_TIME       (FLUSH_TIME),
    .USE_FLUSH        (USE_FLUSH ),
    .RERNW_TIME       (RERNW_TIME)
) arp_inst (
    .clk              (clk                 ),
    .rst              (rst                 ),

    .cfg_ip           (cfg_src_ip          ),
    .cfg_mac          (cfg_src_mac         ),

    .query_ready      (query_ready         ),
    .query_valid      (query_valid         ),
    .query_ip         (query_ip            ),

    .query_resp_mac   (query_resp_mac      ),
    .query_resp_ip    (query_resp_ip       ),
    .query_resp_valid (query_resp_valid    ),

    .s_mac_src_mac    (m_arp_mac_src_mac   ),
    .s_mac_dst_mac    (m_arp_mac_dst_mac   ),
    .s_mac_op_len     (m_arp_mac_op_len    ),
    .s_mac_axis_data  (m_arp_mac_axis_data ),
    .s_mac_axis_valid (m_arp_mac_axis_valid),
    .s_mac_axis_last  (m_arp_mac_axis_last ),
    .s_mac_axis_error (m_arp_mac_axis_error),

    .m_mac_src_mac    (s_arp_mac_src_mac   ),
    .m_mac_dst_mac    (s_arp_mac_dst_mac   ),
    .m_mac_op_len     (s_arp_mac_op_len    ),
    .m_mac_axis_data  (s_arp_mac_axis_data ),
    .m_mac_axis_valid (s_arp_mac_axis_valid),
    .m_mac_axis_ready (s_arp_mac_axis_ready),
    .m_mac_axis_last  (s_arp_mac_axis_last ),
    .m_mac_axis_grant (s_arp_mac_axis_grant),
    .m_mac_axis_req   (s_arp_mac_axis_req  )
);

ip ip_inst(
    .clk                   (clk               ),
    .rst                   (rst               ),
    .cfg_udp_src_port      (cfg_udp_src_port  ),
    .cfg_udp_dst_port      (cfg_udp_dst_port  ), 
    .cfg_src_ip            (cfg_src_ip        ),
    .cfg_dst_ip            (cfg_dst_ip        ),
    .cfg_udp_len           (cfg_udp_len       ),
    .cfg_src_mac           (cfg_src_mac       ),

    .s_mac_src_mac         (m_ip_mac_src_mac   ),
    .s_mac_dst_mac         (m_ip_mac_dst_mac   ),
    .s_mac_op_len          (m_ip_mac_op_len    ),
    .s_mac_axis_data       (m_ip_mac_axis_data ),
    .s_mac_axis_valid      (m_ip_mac_axis_valid),
    .s_mac_axis_last       (m_ip_mac_axis_last ),
    .s_mac_axis_error      (m_ip_mac_axis_error),

    .m_mac_src_mac         (s_ip_mac_src_mac   ),
    .m_mac_dst_mac         (s_ip_mac_dst_mac   ),
    .m_mac_op_len          (s_ip_mac_op_len    ),
    .m_mac_axis_data       (s_ip_mac_axis_data ),
    .m_mac_axis_valid      (s_ip_mac_axis_valid),
    .m_mac_axis_last       (s_ip_mac_axis_last ),
    .m_mac_axis_ready      (s_ip_mac_axis_ready),
    .m_mac_req             (s_ip_mac_axis_req  ),
    .m_mac_grant           (s_ip_mac_axis_grant),

    .m_arp_query_ready     (query_ready        ),
    .m_arp_query_valid     (query_valid        ),
    .m_arp_query_ip        (query_ip           ), 

    .s_arp_query_resp_mac  (query_resp_mac     ),
    .s_arp_query_resp_ip   (query_resp_ip      ),
    .s_arp_query_resp_valid(query_resp_valid   ),

    .m_udp_axis_data       (m_udp_axis_data    ),
    .m_udp_axis_valid      (m_udp_axis_valid   ),
    .m_udp_axis_last       (m_udp_axis_last    ),
    .m_udp_axis_error      (m_udp_axis_error   ),

    .s_udp_axis_data       (s_udp_axis_data    ),
    .s_udp_axis_valid      (s_udp_axis_valid   ),
    .s_udp_axis_last       (s_udp_axis_last    ),
    .s_udp_axis_ready      (s_udp_axis_ready   )
);
    
endmodule