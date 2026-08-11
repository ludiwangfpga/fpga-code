module ip (
    input         clk                   ,
    input         rst                   ,
    input  [15:0] cfg_udp_src_port      ,
    input  [15:0] cfg_udp_dst_port      , 
    input  [31:0] cfg_src_ip            ,
    input  [31:0] cfg_dst_ip            ,
    input  [10:0] cfg_udp_len           ,
    input  [47:0] cfg_src_mac           ,

    input  [47:0] s_mac_src_mac         ,
    input  [47:0] s_mac_dst_mac         ,
    input  [15:0] s_mac_op_len          ,
    input  [ 7:0] s_mac_axis_data       ,
    input         s_mac_axis_valid      ,
    input         s_mac_axis_last       ,
    input         s_mac_axis_error      ,

    output [47:0] m_mac_src_mac         ,
    output [47:0] m_mac_dst_mac         ,
    output [15:0] m_mac_op_len          ,
    output [ 7:0] m_mac_axis_data       ,
    output        m_mac_axis_valid      ,
    output        m_mac_axis_last       ,
    input         m_mac_axis_ready      ,
    output        m_mac_req             ,
    input         m_mac_grant           ,

    input         m_arp_query_ready     ,
    output        m_arp_query_valid     ,
    output [31:0] m_arp_query_ip        , 

    input  [47:0] s_arp_query_resp_mac  ,
    input  [31:0] s_arp_query_resp_ip   ,
    input         s_arp_query_resp_valid,

    output [ 7:0] m_udp_axis_data       ,
    output        m_udp_axis_valid      ,
    output        m_udp_axis_last       ,
    output        m_udp_axis_error      ,

    input  [ 7:0] s_udp_axis_data       ,
    input         s_udp_axis_valid      ,
    input         s_udp_axis_last       ,
    output        s_udp_axis_ready      
);

localparam ICMP = 8'd1 ;
localparam TCP  = 8'd6 ;
localparam UDP  = 8'd17;

wire [ 7:0] m_ip_protocol  ;
wire [ 7:0] m_ip_axis_data ;
wire        m_ip_axis_valid;
wire        m_ip_axis_last ;
wire        m_ip_axis_error;

reg [ 7:0] m_ip_axis_data_ff  ;
reg        m_ip_axis_last_ff  ;
reg        m_ip_axis_error_ff ;
reg        m_ip_udp_axis_valid;

wire [7:0]  s_udp_ip_axis_data ;
wire        s_udp_ip_axis_valid;
wire        s_udp_ip_axis_last ;
wire [15:0] s_udp_ip_axis_len  ;
wire        s_udp_ip_axis_ready;
wire        s_udp_ip_req       ;
wire        s_udp_ip_grant     ;
wire [31:0] s_udp_ip_src_ip    ;
wire [31:0] s_udp_ip_dst_ip    ;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_ip_axis_data_ff   <= 8'b0;
        m_ip_axis_last_ff   <= 1'b0;
        m_ip_axis_error_ff  <= 1'b0;
        m_ip_udp_axis_valid <= 1'b0;
    end
    else begin
        m_ip_axis_data_ff   <= m_ip_axis_data ;
        m_ip_axis_last_ff   <= m_ip_axis_last ;
        m_ip_axis_error_ff  <= m_ip_axis_error;
        m_ip_udp_axis_valid <= m_ip_axis_valid && (m_ip_protocol == UDP);
    end
end

ip_rx ip_rx_inst(
    .clk              (clk             ),
    .rst              (rst             ),

    .cfg_ip           (cfg_src_ip      ),  

    .s_mac_src_mac    (s_mac_src_mac   ),
    .s_mac_dst_mac    (s_mac_dst_mac   ),
    .s_mac_op_len     (s_mac_op_len    ),
    .s_mac_axis_data  (s_mac_axis_data ),
    .s_mac_axis_valid (s_mac_axis_valid),
    .s_mac_axis_last  (s_mac_axis_last ),
    .s_mac_axis_error (s_mac_axis_error),

    .m_ip_protocol    (m_ip_protocol   ),
    .m_ip_axis_data   (m_ip_axis_data  ),
    .m_ip_axis_valid  (m_ip_axis_valid ),
    .m_ip_axis_last   (m_ip_axis_last  ),
    .m_ip_axis_error  (m_ip_axis_error ) 
);

ip_tx ip_tx_inst(
    .clk                   (clk                   ),
    .rst                   (rst                   ),

    .cfg_mac               (cfg_src_mac           ),

    .s_udp_ip_req          (s_udp_ip_req          ),
    .s_udp_ip_grant        (s_udp_ip_grant        ),
    .s_udp_ip_src_ip       (s_udp_ip_src_ip       ),
    .s_udp_ip_dst_ip       (s_udp_ip_dst_ip       ),

    .s_udp_ip_axis_data    (s_udp_ip_axis_data    ),
    .s_udp_ip_axis_valid   (s_udp_ip_axis_valid   ),
    .s_udp_ip_axis_last    (s_udp_ip_axis_last    ),
    .s_udp_ip_axis_len     (s_udp_ip_axis_len     ),
    .s_udp_ip_axis_ready   (s_udp_ip_axis_ready   ),

    .m_mac_src_mac         (m_mac_src_mac         ),
    .m_mac_dst_mac         (m_mac_dst_mac         ),
    .m_mac_op_len          (m_mac_op_len          ),
    .m_mac_axis_data       (m_mac_axis_data       ),
    .m_mac_axis_valid      (m_mac_axis_valid      ),
    .m_mac_axis_last       (m_mac_axis_last       ),
    .m_mac_axis_ready      (m_mac_axis_ready      ),
    .m_mac_req             (m_mac_req             ),
    .m_mac_grant           (m_mac_grant           ),

    .m_arp_query_ready     (m_arp_query_ready     ),
    .m_arp_query_valid     (m_arp_query_valid     ),
    .m_arp_query_ip        (m_arp_query_ip        ), 

    .s_arp_query_resp_mac  (s_arp_query_resp_mac  ),
    .s_arp_query_resp_ip   (s_arp_query_resp_ip   ),
    .s_arp_query_resp_valid(s_arp_query_resp_valid)
);

udp udp_inst(
    .clk              (clk                ),
    .rst              (rst                ),

    .cfg_udp_src_port (cfg_udp_src_port   ),
    .cfg_udp_dst_port (cfg_udp_dst_port   ),
    .cfg_src_ip       (cfg_src_ip         ),
    .cfg_dst_ip       (cfg_dst_ip         ),
    .cfg_udp_len      (cfg_udp_len        ),

    .s_ip_axis_data   (m_ip_axis_data_ff  ),
    .s_ip_axis_valid  (m_ip_udp_axis_valid),
    .s_ip_axis_last   (m_ip_axis_last_ff  ),
    .s_ip_axis_error  (m_ip_axis_error_ff ),

    .m_ip_axis_data   (s_udp_ip_axis_data ),
    .m_ip_axis_valid  (s_udp_ip_axis_valid),
    .m_ip_axis_last   (s_udp_ip_axis_last ),
    .m_ip_axis_len    (s_udp_ip_axis_len  ),
    .m_ip_axis_ready  (s_udp_ip_axis_ready),
    .m_ip_req         (s_udp_ip_req       ),
    .m_ip_grant       (s_udp_ip_grant     ),
    .m_ip_src_ip      (s_udp_ip_src_ip    ),
    .m_ip_dst_ip      (s_udp_ip_dst_ip    ),

    .s_udp_axis_data  (s_udp_axis_data    ),
    .s_udp_axis_valid (s_udp_axis_valid   ),
    .s_udp_axis_last  (s_udp_axis_last    ),
    .s_udp_axis_ready (s_udp_axis_ready   ),

    .m_udp_axis_data  (m_udp_axis_data    ),
    .m_udp_axis_valid (m_udp_axis_valid   ),
    .m_udp_axis_last  (m_udp_axis_last    ),
    .m_udp_axis_error (m_udp_axis_error   )
);
    
endmodule