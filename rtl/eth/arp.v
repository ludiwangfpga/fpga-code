module arp #(
    parameter FLUSH_TIME = 32'hFFFF_FFFF,
    parameter USE_FLUSH  = "TRUE"       ,
    parameter RERNW_TIME = 16'hFFFF   
) (
    input        clk              ,
    input        rst              ,

    input  [31:0] cfg_ip          ,
    input  [47:0] cfg_mac         ,

    output        query_ready     ,
    input         query_valid     ,
    input  [31:0] query_ip        ,

    output [47:0] query_resp_mac  ,
    output [31:0] query_resp_ip   ,
    output        query_resp_valid,

    input  [47:0] s_mac_src_mac   ,
    input  [47:0] s_mac_dst_mac   ,
    input  [15:0] s_mac_op_len    ,
    input  [ 7:0] s_mac_axis_data ,
    input         s_mac_axis_valid,
    input         s_mac_axis_last ,
    input         s_mac_axis_error,

    output [47:0] m_mac_src_mac   ,
    output [47:0] m_mac_dst_mac   ,
    output [15:0] m_mac_op_len    ,
    output [ 7:0] m_mac_axis_data ,
    output        m_mac_axis_valid,
    input         m_mac_axis_ready,
    output        m_mac_axis_last ,
    input         m_mac_axis_grant,
    output        m_mac_axis_req  
);

wire        arp_rx_req_ready        ;
wire        arp_rx_req_arp          ; 
wire [47:0] arp_rx_req_src_mac      ;
wire [47:0] arp_rx_req_dst_mac      ;
wire [31:0] arp_rx_req_src_ip       ;
wire [31:0] arp_rx_req_dst_ip       ;

wire        arp_rx_resp_ready       ;
wire        arp_rx_resp_arp         ; 
wire [47:0] arp_rx_resp_src_mac     ;
wire [47:0] arp_rx_resp_dst_mac     ;
wire [31:0] arp_rx_resp_src_ip      ;
wire [31:0] arp_rx_resp_dst_ip      ;

wire        arp_tx_ready            ;
wire        arp_tx_valid            ;
wire [47:0] arp_tx_mac              ;
wire [31:0] arp_tx_ip               ;
wire [ 1:0] arp_tx_op               ;

arp_rx arp_rx_inst(
    .clk              (clk             ),
    .rst              (rst             ),

    .cfg_ip           (cfg_ip          ),

    .s_mac_src_mac    (s_mac_src_mac   ),
    .s_mac_dst_mac    (s_mac_dst_mac   ),
    .s_mac_op_len     (s_mac_op_len    ),
    .s_mac_axis_data  (s_mac_axis_data ),
    .s_mac_axis_valid (s_mac_axis_valid),
    .s_mac_axis_last  (s_mac_axis_last ),
    .s_mac_axis_error (s_mac_axis_error),

    .req_ready        (arp_rx_req_ready  ),
    .req_arp          (arp_rx_req_arp    ), 
    .req_src_mac      (arp_rx_req_src_mac),
    .req_dst_mac      (arp_rx_req_dst_mac),
    .req_src_ip       (arp_rx_req_src_ip ),
    .req_dst_ip       (arp_rx_req_dst_ip ),

    .resp_ready       (arp_rx_resp_ready  ),
    .resp_arp         (arp_rx_resp_arp    ), 
    .resp_src_mac     (arp_rx_resp_src_mac),
    .resp_dst_mac     (arp_rx_resp_dst_mac),
    .resp_src_ip      (arp_rx_resp_src_ip ),
    .resp_dst_ip      (arp_rx_resp_dst_ip ) 

);

arp_ctrl #(
    .FLUSH_TIME              (FLUSH_TIME),
    .USE_FLUSH               (USE_FLUSH ),
    .RERNW_TIME              (RERNW_TIME)
) arp_ctrl_inst (
    .clk                     (clk),
    .rst                     (rst),

    .arp_rx_req_ready        (arp_rx_req_ready  ),
    .arp_rx_req_arp          (arp_rx_req_arp    ), 
    .arp_rx_req_src_mac      (arp_rx_req_src_mac),
    .arp_rx_req_dst_mac      (arp_rx_req_dst_mac),
    .arp_rx_req_src_ip       (arp_rx_req_src_ip ),
    .arp_rx_req_dst_ip       (arp_rx_req_dst_ip ),

    .arp_rx_resp_ready       (arp_rx_resp_ready  ),
    .arp_rx_resp_arp         (arp_rx_resp_arp    ), 
    .arp_rx_resp_src_mac     (arp_rx_resp_src_mac),
    .arp_rx_resp_dst_mac     (arp_rx_resp_dst_mac),
    .arp_rx_resp_src_ip      (arp_rx_resp_src_ip ),
    .arp_rx_resp_dst_ip      (arp_rx_resp_dst_ip ),

    .query_ready             (query_ready        ),
    .query_valid             (query_valid        ),
    .query_ip                (query_ip           ),

    .query_resp_mac          (query_resp_mac     ),
    .query_resp_ip           (query_resp_ip      ),
    .query_resp_valid        (query_resp_valid   ),

    .arp_tx_ready            (arp_tx_ready       ),
    .arp_tx_valid            (arp_tx_valid       ),
    .arp_tx_mac              (arp_tx_mac         ),
    .arp_tx_ip               (arp_tx_ip          ),
    .arp_tx_op               (arp_tx_op          ) 
);

arp_tx arp_tx_inst(
    .clk               (clk            ),
    .rst               (rst            ),

    .cfg_ip            (cfg_ip         ),
    .cfg_mac           (cfg_mac        ),

    .arp_tx_valid      (arp_tx_valid   ),
    .arp_tx_ready      (arp_tx_ready   ),
    .arp_tx_ip         (arp_tx_ip      ),
    .arp_tx_mac        (arp_tx_mac     ),
    .arp_tx_op         (arp_tx_op      ),

    .m_mac_src_mac     (m_mac_src_mac   ),
    .m_mac_dst_mac     (m_mac_dst_mac   ),
    .m_mac_op_len      (m_mac_op_len    ),
    .m_mac_axis_data   (m_mac_axis_data ),
    .m_mac_axis_valid  (m_mac_axis_valid),
    .m_mac_axis_ready  (m_mac_axis_ready),
    .m_mac_axis_last   (m_mac_axis_last ),
    .m_mac_axis_grant  (m_mac_axis_grant),
    .m_mac_axis_req    (m_mac_axis_req  )

);
    
endmodule