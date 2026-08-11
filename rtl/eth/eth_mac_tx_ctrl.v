module eth_mac_tx_ctrl (
    input         clk              ,
    input         rst              ,
    
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

reg ping_r;
reg ping_w;

wire [47:0] s_mac_src_mac_ping    ;
wire [47:0] s_mac_dst_mac_ping    ;
wire [15:0] s_mac_op_len_ping     ;     
wire [ 7:0] s_mac_axis_data_ping  ;
wire        s_mac_axis_valid_ping ;
wire        s_mac_axis_ready_ping ;
wire        s_mac_axis_last_ping  ;

wire [ 7:0] m_eth_axis_data_ping  ;
wire        m_eth_axis_valid_ping ;
wire        m_eth_axis_ready_ping ;
wire        m_eth_axis_last_ping  ;

wire [47:0] s_mac_src_mac_pang    ;
wire [47:0] s_mac_dst_mac_pang    ;
wire [15:0] s_mac_op_len_pang     ;     
wire [ 7:0] s_mac_axis_data_pang  ;
wire        s_mac_axis_valid_pang ;
wire        s_mac_axis_ready_pang ;
wire        s_mac_axis_last_pang  ;

wire [ 7:0] m_eth_axis_data_pang  ;
wire        m_eth_axis_valid_pang ;
wire        m_eth_axis_ready_pang ;
wire        m_eth_axis_last_pang  ;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        ping_w <= 1'b0;
    end
    else begin
        if((s_mac_axis_valid == 1'b1) && (s_mac_axis_last == 1'b1))begin
            ping_w <= !ping_w;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        ping_r <= 1'b0;
    end
    else begin
        if((m_eth_axis_valid == 1'b1) && (m_eth_axis_last == 1'b1))begin
            ping_r <= !ping_r;
        end
    end
end



assign s_mac_src_mac_ping    = ping_w == 1'b0 ? s_mac_src_mac    : 48'b0;
assign s_mac_dst_mac_ping    = ping_w == 1'b0 ? s_mac_dst_mac    : 48'b0;
assign s_mac_op_len_ping     = ping_w == 1'b0 ? s_mac_op_len     : 16'b0;
assign s_mac_axis_data_ping  = ping_w == 1'b0 ? s_mac_axis_data  :  8'b0;
assign s_mac_axis_valid_ping = ping_w == 1'b0 ? s_mac_axis_valid :  1'b0;
assign s_mac_axis_last_ping  = ping_w == 1'b0 ? s_mac_axis_last  :  1'b0;

assign s_mac_src_mac_pang    = ping_w == 1'b1 ? s_mac_src_mac    : 48'b0;
assign s_mac_dst_mac_pang    = ping_w == 1'b1 ? s_mac_dst_mac    : 48'b0;
assign s_mac_op_len_pang     = ping_w == 1'b1 ? s_mac_op_len     : 16'b0;
assign s_mac_axis_data_pang  = ping_w == 1'b1 ? s_mac_axis_data  :  8'b0;
assign s_mac_axis_valid_pang = ping_w == 1'b1 ? s_mac_axis_valid :  1'b0;
assign s_mac_axis_last_pang  = ping_w == 1'b1 ? s_mac_axis_last  :  1'b0;

assign s_mac_axis_ready      = ping_w == 1'b0 ? s_mac_axis_ready_ping : s_mac_axis_ready_pang;

assign m_eth_axis_data       = ping_r == 1'b0 ? m_eth_axis_data_ping  : m_eth_axis_data_pang ;
assign m_eth_axis_valid      = ping_r == 1'b0 ? m_eth_axis_valid_ping : m_eth_axis_valid_pang;
assign m_eth_axis_last       = ping_r == 1'b0 ? m_eth_axis_last_ping  : m_eth_axis_last_pang ;

assign m_eth_axis_ready_ping = ping_r == 1'b0 ? m_eth_axis_ready : 1'b0;
assign m_eth_axis_ready_pang = ping_r == 1'b1 ? m_eth_axis_ready : 1'b0;



eth_mac_tx eth_mac_tx_ping(
    .clk              (clk                  ),
    .rst              (rst                  ),
    
    .s_mac_src_mac    (s_mac_src_mac_ping   ),
    .s_mac_dst_mac    (s_mac_dst_mac_ping   ),
    .s_mac_op_len     (s_mac_op_len_ping    ),     
    .s_mac_axis_data  (s_mac_axis_data_ping ),
    .s_mac_axis_valid (s_mac_axis_valid_ping),
    .s_mac_axis_ready (s_mac_axis_ready_ping),
    .s_mac_axis_last  (s_mac_axis_last_ping ),

    .m_eth_axis_data  (m_eth_axis_data_ping ),
    .m_eth_axis_valid (m_eth_axis_valid_ping),
    .m_eth_axis_ready (m_eth_axis_ready_ping),
    .m_eth_axis_last  (m_eth_axis_last_ping ) 
);

eth_mac_tx eth_mac_tx_pang(
    .clk              (clk                  ),
    .rst              (rst                  ),
    
    .s_mac_src_mac    (s_mac_src_mac_pang   ),
    .s_mac_dst_mac    (s_mac_dst_mac_pang   ),
    .s_mac_op_len     (s_mac_op_len_pang    ),     
    .s_mac_axis_data  (s_mac_axis_data_pang ),
    .s_mac_axis_valid (s_mac_axis_valid_pang),
    .s_mac_axis_ready (s_mac_axis_ready_pang),
    .s_mac_axis_last  (s_mac_axis_last_pang ),

    .m_eth_axis_data  (m_eth_axis_data_pang ),
    .m_eth_axis_valid (m_eth_axis_valid_pang),
    .m_eth_axis_ready (m_eth_axis_ready_pang),
    .m_eth_axis_last  (m_eth_axis_last_pang ) 
);
    
endmodule