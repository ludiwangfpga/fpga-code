module eth_rx_ctrl (
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
    output        m_ip_mac_axis_error  
);

localparam ARP = 4'd1;
localparam IP  = 4'd2;

reg [3:0] op;
reg [47:0] s_mac_src_mac_ff      ;
reg [47:0] s_mac_dst_mac_ff      ;
reg [15:0] s_mac_op_len_ff       ;
reg [ 7:0] s_mac_axis_data_ff    ;
reg        s_mac_axis_valid_ff   ;
reg        s_mac_axis_last_ff    ;
reg        s_mac_axis_error_ff   ;

reg [47:0] s_arp_mac_src_mac_r   ;
reg [47:0] s_arp_mac_dst_mac_r   ;
reg [15:0] s_arp_mac_op_len_r    ;
reg [ 7:0] s_arp_mac_axis_data_r ;
reg        s_arp_mac_axis_valid_r;
reg        s_arp_mac_axis_last_r ;
reg        s_arp_mac_axis_error_r;

reg [47:0] s_ip_mac_src_mac_r    ;
reg [47:0] s_ip_mac_dst_mac_r    ;
reg [15:0] s_ip_mac_op_len_r     ;
reg [ 7:0] s_ip_mac_axis_data_r  ;
reg        s_ip_mac_axis_valid_r ;
reg        s_ip_mac_axis_last_r  ;
reg        s_ip_mac_axis_error_r ;
always @(posedge clk ) begin
    if(rst == 1'b1)begin
        op <= 4'd0;
    end
    else begin
        if(s_mac_axis_valid == 1'b1)begin
            case(s_mac_op_len)
            16'h0806: begin
                op <= ARP;
            end
            16'h0800: begin
                op <= IP;
            end
            endcase
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_mac_src_mac_ff    <= 48'd0;
        s_mac_dst_mac_ff    <= 48'd0;
        s_mac_op_len_ff     <= 16'd0;
        s_mac_axis_data_ff  <=  8'd0;
        s_mac_axis_valid_ff <=  1'b0;
        s_mac_axis_last_ff  <=  1'b0;
        s_mac_axis_error_ff <=  1'b0;
    end
    else begin
        s_mac_src_mac_ff    <= s_mac_src_mac   ;
        s_mac_dst_mac_ff    <= s_mac_dst_mac   ;
        s_mac_op_len_ff     <= s_mac_op_len    ;
        s_mac_axis_data_ff  <= s_mac_axis_data ;
        s_mac_axis_valid_ff <= s_mac_axis_valid;
        s_mac_axis_last_ff  <= s_mac_axis_last ;
        s_mac_axis_error_ff <= s_mac_axis_error;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_arp_mac_src_mac_r    <= 48'd0;
        s_arp_mac_dst_mac_r    <= 48'd0;
        s_arp_mac_op_len_r     <= 16'd0;
        s_arp_mac_axis_data_r  <=  8'd0;
        s_arp_mac_axis_valid_r <=  1'b0;
        s_arp_mac_axis_last_r  <=  1'b0;
        s_arp_mac_axis_error_r <=  1'b0;
    end
    else begin
        s_arp_mac_src_mac_r    <= s_mac_src_mac_ff   ;
        s_arp_mac_dst_mac_r    <= s_mac_dst_mac_ff   ;
        s_arp_mac_op_len_r     <= s_mac_op_len_ff    ;
        s_arp_mac_axis_data_r  <= s_mac_axis_data_ff ;
        s_arp_mac_axis_valid_r <= s_mac_axis_valid_ff && (op == ARP);
        s_arp_mac_axis_last_r  <= s_mac_axis_last_ff ;
        s_arp_mac_axis_error_r <= s_mac_axis_error_ff;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ip_mac_src_mac_r    <= 48'd0;
        s_ip_mac_dst_mac_r    <= 48'd0;
        s_ip_mac_op_len_r     <= 16'd0;
        s_ip_mac_axis_data_r  <=  8'd0;
        s_ip_mac_axis_valid_r <=  1'b0;
        s_ip_mac_axis_last_r  <=  1'b0;
        s_ip_mac_axis_error_r <=  1'b0;
    end
    else begin
        s_ip_mac_src_mac_r    <= s_mac_src_mac_ff   ;
        s_ip_mac_dst_mac_r    <= s_mac_dst_mac_ff   ;
        s_ip_mac_op_len_r     <= s_mac_op_len_ff    ;
        s_ip_mac_axis_data_r  <= s_mac_axis_data_ff ;
        s_ip_mac_axis_valid_r <= s_mac_axis_valid_ff && (op == IP);
        s_ip_mac_axis_last_r  <= s_mac_axis_last_ff ;
        s_ip_mac_axis_error_r <= s_mac_axis_error_ff;
    end
end

assign m_arp_mac_src_mac    = s_arp_mac_src_mac_r   ;
assign m_arp_mac_dst_mac    = s_arp_mac_dst_mac_r   ;
assign m_arp_mac_op_len     = s_arp_mac_op_len_r    ;
assign m_arp_mac_axis_data  = s_arp_mac_axis_data_r ;
assign m_arp_mac_axis_valid = s_arp_mac_axis_valid_r;
assign m_arp_mac_axis_last  = s_arp_mac_axis_last_r ;
assign m_arp_mac_axis_error = s_arp_mac_axis_error_r;

assign m_ip_mac_src_mac     = s_ip_mac_src_mac_r    ;
assign m_ip_mac_dst_mac     = s_ip_mac_dst_mac_r    ;
assign m_ip_mac_op_len      = s_ip_mac_op_len_r     ;
assign m_ip_mac_axis_data   = s_ip_mac_axis_data_r  ;
assign m_ip_mac_axis_valid  = s_ip_mac_axis_valid_r ;
assign m_ip_mac_axis_last   = s_ip_mac_axis_last_r  ;
assign m_ip_mac_axis_error  = s_ip_mac_axis_error_r ;
    
endmodule