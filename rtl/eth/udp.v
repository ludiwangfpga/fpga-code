module udp (
    input         clk              ,
    input         rst              ,

    input  [15:0] cfg_udp_src_port ,
    input  [15:0] cfg_udp_dst_port ,
    input  [31:0] cfg_src_ip       ,
    input  [31:0] cfg_dst_ip       ,
    input  [10:0] cfg_udp_len      ,

    input  [ 7:0] s_ip_axis_data   ,
    input         s_ip_axis_valid  ,
    input         s_ip_axis_last   ,
    input         s_ip_axis_error  ,

    output [7:0]  m_ip_axis_data   ,
    output        m_ip_axis_valid  ,
    output        m_ip_axis_last   ,
    output [15:0] m_ip_axis_len    ,
    input         m_ip_axis_ready  ,
    output        m_ip_req         ,
    input         m_ip_grant       ,
    output [31:0] m_ip_src_ip      ,
    output [31:0] m_ip_dst_ip      ,

    input  [ 7:0] s_udp_axis_data  ,
    input         s_udp_axis_valid ,
    input         s_udp_axis_last  ,
    output        s_udp_axis_ready ,

    output [ 7:0] m_udp_axis_data  ,
    output        m_udp_axis_valid ,
    output        m_udp_axis_last  ,
    output        m_udp_axis_error     
);


wire [ 7:0] udp_ping_data    ;
wire        udp_ping_valid   ;
wire        udp_ping_last    ;
wire        udp_ping_ready   ;

wire [ 7:0] udp_pang_data    ;
wire        udp_pang_valid   ;
wire        udp_pang_last    ;
wire        udp_pang_ready   ;

wire [7:0]  ip_ping_data     ;
wire        ip_ping_valid    ;
wire        ip_ping_last     ;
wire [15:0] ip_ping_len      ;
wire [31:0] ip_ping_src_ip   ;
wire [31:0] ip_ping_dst_ip   ;
wire        ip_ping_ready    ;

wire [7:0]  ip_pang_data     ;
wire        ip_pang_valid    ;
wire        ip_pang_last     ;
wire [15:0] ip_pang_len      ;
wire [31:0] ip_pang_src_ip   ;
wire [31:0] ip_pang_dst_ip   ;
wire        ip_pang_ready    ;

wire        udp_ping_req     ;
wire        udp_ping_grant   ;
wire        udp_pang_req     ;
wire        udp_pang_grant   ;

udp_rx udp_rx_inst(
    .clk              (clk              ),
    .rst              (rst              ),

    .cfg_udp_src_port (cfg_udp_src_port ),
    .cfg_udp_dst_port (cfg_udp_dst_port ),

    .s_ip_axis_data   (s_ip_axis_data   ),
    .s_ip_axis_valid  (s_ip_axis_valid  ),
    .s_ip_axis_last   (s_ip_axis_last   ),
    .s_ip_axis_error  (s_ip_axis_error  ),

    .m_udp_axis_data  (m_udp_axis_data  ),
    .m_udp_axis_valid (m_udp_axis_valid ),
    .m_udp_axis_last  (m_udp_axis_last  ),
    .m_udp_axis_error (m_udp_axis_error )  
);

udp_tx udp_tx_ping(
    .clk              (clk              ),
    .rst              (rst              ),

    .cfg_udp_src_port (cfg_udp_src_port ),
    .cfg_udp_dst_port (cfg_udp_dst_port ),
    .cfg_src_ip       (cfg_src_ip       ),
    .cfg_dst_ip       (cfg_dst_ip       ),
    .cfg_udp_len      (cfg_udp_len      ),

    .s_udp_axis_data  (udp_ping_data    ),
    .s_udp_axis_valid (udp_ping_valid   ),
    .s_udp_axis_last  (udp_ping_last    ),
    .s_udp_axis_ready (udp_ping_ready   ),

    .m_ip_axis_data   (ip_ping_data     ),
    .m_ip_axis_valid  (ip_ping_valid    ),
    .m_ip_axis_last   (ip_ping_last     ),
    .m_ip_axis_len    (ip_ping_len      ),
    .m_ip_axis_ready  (ip_ping_ready    ),

    .m_ip_req         (udp_ping_req     ),
    .m_ip_grant       (udp_ping_grant   ),
    .m_ip_src_ip      (ip_ping_src_ip   ),
    .m_ip_dst_ip      (ip_ping_dst_ip   )

);

udp_tx udp_tx_pang(
    .clk              (clk              ),
    .rst              (rst              ),

    .cfg_udp_src_port (cfg_udp_src_port ),
    .cfg_udp_dst_port (cfg_udp_dst_port ),
    .cfg_src_ip       (cfg_src_ip       ),
    .cfg_dst_ip       (cfg_dst_ip       ),
    .cfg_udp_len      (cfg_udp_len      ),

    .s_udp_axis_data  (udp_pang_data    ),
    .s_udp_axis_valid (udp_pang_valid   ),
    .s_udp_axis_last  (udp_pang_last    ),
    .s_udp_axis_ready (udp_pang_ready   ),

    .m_ip_axis_data   (ip_pang_data     ),
    .m_ip_axis_valid  (ip_pang_valid    ),
    .m_ip_axis_last   (ip_pang_last     ),
    .m_ip_axis_ready  (ip_pang_ready    ),
    .m_ip_axis_len    (ip_pang_len      ),

    .m_ip_req         (udp_pang_req     ),
    .m_ip_grant       (udp_pang_grant   ),
    .m_ip_src_ip      (ip_pang_src_ip   ),
    .m_ip_dst_ip      (ip_pang_dst_ip   )

);

udp_tx_ctrl udp_tx_ctrl_inst(
    .clk              (clk              ),
    .rst              (rst              ),

    .s_udp_axis_data  (s_udp_axis_data  ),
    .s_udp_axis_valid (s_udp_axis_valid ),
    .s_udp_axis_last  (s_udp_axis_last  ),
    .s_udp_axis_ready (s_udp_axis_ready ),

    .m_ip_axis_data   (m_ip_axis_data   ),
    .m_ip_axis_valid  (m_ip_axis_valid  ),
    .m_ip_axis_last   (m_ip_axis_last   ),
    .m_ip_axis_len    (m_ip_axis_len    ),
    .m_ip_axis_ready  (m_ip_axis_ready  ),

    .m_ip_req         (m_ip_req         ),       
    .m_ip_grant       (m_ip_grant       ), 
    .m_ip_src_ip      (m_ip_src_ip      ),
    .m_ip_dst_ip      (m_ip_dst_ip      ),      

    .udp_ping_data    (udp_ping_data    ),
    .udp_ping_valid   (udp_ping_valid   ),
    .udp_ping_last    (udp_ping_last    ),
    .udp_ping_ready   (udp_ping_ready   ),

    .udp_pang_data    (udp_pang_data    ),
    .udp_pang_valid   (udp_pang_valid   ),
    .udp_pang_last    (udp_pang_last    ),
    .udp_pang_ready   (udp_pang_ready   ),

    .udp_ping_req     (udp_ping_req     ),
    .udp_ping_grant   (udp_ping_grant   ),

    .udp_pang_req     (udp_pang_req     ),
    .udp_pang_grant   (udp_pang_grant   ),

    .ip_ping_data     (ip_ping_data     ),
    .ip_ping_valid    (ip_ping_valid    ),
    .ip_ping_last     (ip_ping_last     ),
    .ip_ping_len      (ip_ping_len      ),
    .ip_ping_src_ip   (ip_ping_src_ip   ),
    .ip_ping_dst_ip   (ip_ping_dst_ip   ),
    .ip_ping_ready    (ip_ping_ready    ),

    .ip_pang_data     (ip_pang_data     ),
    .ip_pang_valid    (ip_pang_valid    ),
    .ip_pang_last     (ip_pang_last     ),
    .ip_pang_len      (ip_pang_len      ),
    .ip_pang_src_ip   (ip_pang_src_ip   ),
    .ip_pang_dst_ip   (ip_pang_dst_ip   ),
    .ip_pang_ready    (ip_pang_ready    )
);
    
endmodule