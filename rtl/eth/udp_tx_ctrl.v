module udp_tx_ctrl (
    input         clk              ,
    input         rst              ,

    input  [ 7:0] s_udp_axis_data  ,
    input         s_udp_axis_valid ,
    input         s_udp_axis_last  ,
    output        s_udp_axis_ready ,

    output [7:0]  m_ip_axis_data   ,
    output        m_ip_axis_valid  ,
    output        m_ip_axis_last   ,
    output [15:0] m_ip_axis_len    ,
    input         m_ip_axis_ready  ,

    output        m_ip_req         ,
    input         m_ip_grant       ,
    output [31:0] m_ip_src_ip      ,
    output [31:0] m_ip_dst_ip      ,

    output [ 7:0] udp_ping_data    ,
    output        udp_ping_valid   ,
    output        udp_ping_last    ,
    input         udp_ping_ready   ,

    output [ 7:0] udp_pang_data    ,
    output        udp_pang_valid   ,
    output        udp_pang_last    ,
    input         udp_pang_ready   ,

    input         udp_ping_req     ,
    output        udp_ping_grant   ,

    input         udp_pang_req     ,
    output        udp_pang_grant   ,

    input  [7:0]  ip_ping_data     ,
    input         ip_ping_valid    ,
    input         ip_ping_last     ,
    input  [15:0] ip_ping_len      ,
    input  [31:0] ip_ping_src_ip   ,
    input  [31:0] ip_ping_dst_ip   ,
    output        ip_ping_ready    ,

    input  [7:0]  ip_pang_data     ,
    input         ip_pang_valid    ,
    input         ip_pang_last     ,
    input  [15:0] ip_pang_len      ,
    input  [31:0] ip_pang_src_ip   ,
    input  [31:0] ip_pang_dst_ip   ,
    output        ip_pang_ready    
);

reg udp_ping;
reg ip_ping ;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        udp_ping <= 1'b0;
    end
    else begin
        if((s_udp_axis_valid == 1'b1) && (s_udp_axis_ready == 1'b1) && (s_udp_axis_last == 1'b1))begin
            udp_ping <= ~udp_ping;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        ip_ping <= 1'b0;
    end
    else begin
        if((m_ip_axis_valid == 1'b1) && (m_ip_axis_last == 1'b1))begin
            ip_ping <= ~ip_ping;
        end
    end
end

assign udp_ping_data  = udp_ping == 1'b0 ? s_udp_axis_data  : 8'b0;
assign udp_ping_valid = udp_ping == 1'b0 ? s_udp_axis_valid : 1'b0;
assign udp_ping_last  = udp_ping == 1'b0 ? s_udp_axis_last  : 1'b0;

assign udp_pang_data  = udp_ping == 1'b1 ? s_udp_axis_data  : 8'b0;
assign udp_pang_valid = udp_ping == 1'b1 ? s_udp_axis_valid : 1'b0;
assign udp_pang_last  = udp_ping == 1'b1 ? s_udp_axis_last  : 1'b0;

assign s_udp_axis_ready = udp_ping == 1'b0 ? udp_ping_ready : udp_pang_ready;

assign udp_ping_grant = ip_ping == 1'b0 ? m_ip_grant : 1'b0;
assign udp_pang_grant = ip_ping == 1'b1 ? m_ip_grant : 1'b0;

assign m_ip_req = ip_ping == 1'b0 ? udp_ping_req : udp_pang_req;

assign m_ip_axis_data  = ip_ping == 1'b0 ? ip_ping_data  : ip_pang_data ;
assign m_ip_axis_valid = ip_ping == 1'b0 ? ip_ping_valid : ip_pang_valid;
assign m_ip_axis_last  = ip_ping == 1'b0 ? ip_ping_last  : ip_pang_last ;
assign m_ip_axis_len   = ip_ping == 1'b0 ? ip_ping_len   : ip_pang_len  ;

assign m_ip_src_ip = ip_ping == 1'b0 ? ip_ping_src_ip : ip_pang_src_ip;
assign m_ip_dst_ip = ip_ping == 1'b0 ? ip_ping_dst_ip : ip_pang_dst_ip;

assign ip_ping_ready = ip_ping == 1'b0 ? m_ip_axis_ready : 1'b0;
assign ip_pang_ready = ip_ping == 1'b1 ? m_ip_axis_ready : 1'b0;

    
endmodule