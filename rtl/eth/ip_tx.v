module ip_tx (
    input         clk                   ,
    input         rst                   ,

    input  [47:0] cfg_mac               ,

    input         s_udp_ip_req          ,
    output        s_udp_ip_grant        ,
    input  [31:0] s_udp_ip_src_ip       ,
    input  [31:0] s_udp_ip_dst_ip       ,

    input  [ 7:0] s_udp_ip_axis_data    ,
    input         s_udp_ip_axis_valid   ,
    input         s_udp_ip_axis_last    ,
    input  [15:0] s_udp_ip_axis_len     ,
    output        s_udp_ip_axis_ready   ,

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
    input         s_arp_query_resp_valid
);

localparam REQ_NUM = 2;
localparam ICMP = 8'd1 ;
localparam TCP  = 8'd6 ;
localparam UDP  = 8'd17;

genvar i;

localparam IDLE     = 5'b00001;
localparam GRANT    = 5'b00010;
localparam UDP_MAC  = 5'b00100;
localparam UDP_DATA = 5'b01000;
localparam WAIT     = 5'b10000;

reg [4:0] curr_state;
reg [4:0] next_state;

reg m_mac_req_r;

wire [REQ_NUM - 1:0] req;

reg  [REQ_NUM - 1:0] grant_ff;
wire [REQ_NUM - 1:0] grant   ;

reg        m_arp_query_valid_r ;
reg [31:0] m_arp_query_ip_r    ;

reg        s_udp_ip_axis_ready_r;

reg [ 7:0] s_ip_axis_data_ff [23:0]; 
reg [23:0] s_ip_axis_valid_ff      ;
reg [23:0] s_ip_axis_last_ff       ;

reg [ 7:0] s_ip_protocol_r;
reg [15:0] s_ip_len_r     ;
reg [31:0] s_ip_src_ip_r  ;
reg [31:0] s_ip_dst_ip_r  ;
reg [47:0] dst_mac        ;

reg [16:0] fcs;
reg [16:0] fcs_src_ip;
reg [16:0] fcs_dst_ip;
reg [16:0] fcs_len_index;
reg [16:0] fcs_protocol;

reg [ 7:0] index;
reg [ 4:0] send_cnt;

reg [159:0] data_temp;

reg [47:0] m_mac_src_mac_r   ;   
reg [47:0] m_mac_dst_mac_r   ;   
reg [15:0] m_mac_op_len_r    ;    
reg [ 7:0] m_mac_axis_data_r ; 
reg        m_mac_axis_valid_r;
reg        m_mac_axis_last_r ; 

assign req = (curr_state == IDLE) ? s_udp_ip_req : 1'b0;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        curr_state <= IDLE;
    end
    else begin
        curr_state <= next_state;
    end
end

always @(*)begin
    case (curr_state)
        IDLE: begin
            if(|req)begin
                next_state = GRANT;
            end
            else begin
                next_state = IDLE;
            end
        end 
        GRANT: begin
            if(grant_ff[0] == 1'b1)begin
                next_state = UDP_MAC;
            end
            else begin
                next_state = UDP_MAC;
            end         
        end
        UDP_MAC: begin
            if((s_arp_query_resp_valid == 1'b1) && (s_arp_query_resp_ip == s_ip_dst_ip_r))begin
                next_state = WAIT;
            end
            else begin
                next_state = UDP_MAC;
            end
        end
        UDP_DATA: begin
            if((m_mac_axis_last == 1'b1) && (m_mac_axis_valid == 1'b1))begin
                next_state = IDLE;
            end
            else begin
                next_state = UDP_DATA;
            end
        end
        WAIT: begin
            if(m_mac_grant == 1'b1)begin
                if(grant_ff[0] == 1'b1)begin
                    next_state = UDP_DATA;
                end
                else begin
                    next_state = UDP_DATA;
                end 
            end
            else begin
                next_state = WAIT;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end

assign s_udp_ip_grant = (m_mac_grant == 1'b1) && (grant_ff[0] == 1'b1);

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_udp_ip_axis_ready_r <= 1'b1;
    end
    else begin
        if(curr_state == UDP_DATA)begin
            s_udp_ip_axis_ready_r <= m_mac_axis_ready;
        end
        else begin
            s_udp_ip_axis_ready_r <= 1'b0;
        end
    end
end

assign s_udp_ip_axis_ready = s_udp_ip_axis_ready_r & m_mac_axis_ready;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_mac_req_r <= 1'b0;
    end
    else begin
        if(curr_state == WAIT)begin
            if((m_mac_req_r == 1'b1) && (m_mac_grant == 1'b1))begin
                m_mac_req_r <= 1'b0;
            end
            else begin
                m_mac_req_r <= 1'b1;
            end
        end
        else begin
            m_mac_req_r <= 1'b0;
        end
    end
end

assign m_mac_req = m_mac_req_r;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        grant_ff <= {REQ_NUM{1'b0}};
    end
    else begin
        if(curr_state == IDLE)begin
            grant_ff <= grant;
        end
        else begin
            if(m_mac_grant == 1'b1)begin
                grant_ff <= {REQ_NUM{1'b0}};
            end
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ip_src_ip_r   <= 31'd0;
        s_ip_dst_ip_r   <= 31'd0;
    end
    else begin
        if(grant_ff[0] == 1'b1)begin
            s_ip_src_ip_r   <= s_udp_ip_src_ip;
            s_ip_dst_ip_r   <= s_udp_ip_dst_ip;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_arp_query_valid_r <=  1'b0;
        m_arp_query_ip_r    <= 31'b0;
    end
    else begin
        if((curr_state == GRANT) && (next_state == UDP_MAC))begin
            if(grant_ff[0] == 1'b1)begin
                m_arp_query_valid_r <=  1'b1;
                m_arp_query_ip_r    <= s_udp_ip_dst_ip;
            end
        end
        else begin
            if(curr_state == UDP_MAC)begin
                if((s_arp_query_resp_valid == 1'b1) && (s_arp_query_resp_ip != s_ip_dst_ip_r))begin
                    m_arp_query_valid_r <=  1'b1;
                    m_arp_query_ip_r    <= s_udp_ip_dst_ip;
                end
            end
            if((m_arp_query_valid == 1'b1) && (m_arp_query_ready == 1'b1))begin
                m_arp_query_valid_r <=  1'b0;
            end
        end
    end
end

assign m_arp_query_valid = m_arp_query_valid_r;
assign m_arp_query_ip    = m_arp_query_ip_r   ;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ip_axis_data_ff[0] <= 8'b0;
        s_ip_axis_valid_ff   <= 24'b0;
        s_ip_axis_last_ff    <= 24'b0;
    end
    else begin
        if(curr_state == UDP_DATA)begin
            s_ip_axis_data_ff[0] <= s_udp_ip_axis_data ;
            s_ip_axis_valid_ff   <= {s_ip_axis_valid_ff[22:0], s_udp_ip_axis_valid};
            s_ip_axis_last_ff    <= {s_ip_axis_last_ff [22:0], s_udp_ip_axis_last };
        end      
    end
end

generate
    for (i = 0; i < 23; i = i + 1) begin : LOOP_UDP_DATA
        always @(posedge clk ) begin
            s_ip_axis_data_ff[i + 1] <= s_ip_axis_data_ff[i];
        end
    end
endgenerate

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ip_protocol_r <=  8'd0;
        s_ip_len_r      <= 16'd0;      
    end
    else begin
        if((s_udp_ip_axis_valid == 1'b1) && (s_ip_axis_valid_ff[0] == 1'b0))begin
            s_ip_protocol_r <= UDP;
            s_ip_len_r      <= s_udp_ip_axis_len + 16'd20;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        index <= 8'd0;
    end
    else begin
        if((curr_state == IDLE) && (next_state == GRANT))begin
            index <= index + 8'd1;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        fcs_src_ip <= 17'd0;
        fcs_dst_ip <= 17'd0;
        fcs_len_index <= 17'd0;
        fcs_protocol  <= 17'd0;
    end
    else begin
        if(send_cnt == 5'd1)begin
            fcs_src_ip <= s_ip_src_ip_r[15:0] + s_ip_src_ip_r[31:16];
            fcs_dst_ip <= s_ip_dst_ip_r[15:0] + s_ip_dst_ip_r[31:16];
            fcs_len_index <= index + s_ip_len_r;
            fcs_protocol  <= {8'h40, s_ip_protocol_r};
        end
        else if(send_cnt == 5'd2)begin
            fcs_src_ip <= fcs_src_ip[15:0] + fcs_src_ip[16];
            fcs_dst_ip <= fcs_dst_ip[15:0] + fcs_dst_ip[16];
            fcs_len_index <= fcs_len_index[15:0] + fcs_len_index[16];
            fcs_protocol  <= fcs_protocol;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        fcs <= 17'h8500; // 0x4500 + 0x4000
    end
    else begin
        if(curr_state == IDLE)begin
            fcs <= 17'h8500;
        end
        else if(send_cnt == 5'd3)begin
            fcs <= fcs[15:0] + fcs_src_ip + fcs[16];
        end
        else if(send_cnt == 5'd4)begin
            fcs <= fcs[15:0] + fcs_dst_ip + fcs[16];
        end
        else if(send_cnt == 5'd5)begin
            fcs <= fcs[15:0] + fcs_len_index + fcs[16];
        end
        else if(send_cnt == 5'd6)begin
            fcs <= fcs[15:0] + fcs_protocol + fcs[16];
        end
        else if(send_cnt == 5'd7)begin
            fcs <= ~(fcs[15:0] + fcs[16]);
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        dst_mac <= 48'd0;
    end
    else begin
        if((s_arp_query_resp_valid == 1'b1) && (s_arp_query_resp_ip == s_ip_dst_ip_r))begin
            dst_mac <= s_arp_query_resp_mac;
        end
    end
end


always @(posedge clk ) begin
    m_mac_src_mac_r <= cfg_mac;
    m_mac_dst_mac_r <= dst_mac;
    m_mac_op_len_r  <= 16'h0800;
    
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        send_cnt <= 5'd0;
    end
    else begin
        if((s_ip_axis_valid_ff[1] == 1'b1) && (send_cnt == 5'd0))begin
            send_cnt <= 5'd1;
        end
        else if((send_cnt < 5'd23) && (send_cnt > 5'd0))begin
            send_cnt <= send_cnt + 5'd1;
        end
        else if((m_mac_axis_valid == 1'b1) && (m_mac_axis_last == 1'b1))begin
            send_cnt <= 5'd0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        data_temp          <= 159'd0;
        m_mac_axis_data_r  <= 8'd0;
        m_mac_axis_valid_r <= 1'd0;
        m_mac_axis_last_r  <= 1'd0;
    end
    else begin
        if((s_ip_axis_valid_ff[0] == 1'b1) && (s_ip_axis_valid_ff[1] == 1'b0))begin
            data_temp <= {16'h4500, s_ip_len_r, {8'd0, index}, 16'h4000, {8'h40, s_ip_protocol_r}, 16'h0000, s_ip_src_ip_r, s_ip_dst_ip_r};
        end
        if((send_cnt >= 5'd1) && (send_cnt < 5'd21))begin
            data_temp <= data_temp << 4'd8;
            if(send_cnt == 5'd11)begin
                m_mac_axis_data_r  <= fcs[15:8];
            end
            else if(send_cnt == 5'd12)begin
                m_mac_axis_data_r  <= fcs[7:0];
            end
            else begin
                m_mac_axis_data_r  <= data_temp[159 -: 8];
            end
            m_mac_axis_valid_r <= 1'd1;
            m_mac_axis_last_r  <= 1'd0;
        end
        else begin
            m_mac_axis_data_r  <= s_ip_axis_data_ff [23];
            m_mac_axis_valid_r <= s_ip_axis_valid_ff[23];
            m_mac_axis_last_r  <= s_ip_axis_last_ff [23];
        end
    end
end

assign m_mac_src_mac    = m_mac_src_mac_r   ;
assign m_mac_dst_mac    = m_mac_dst_mac_r   ;
assign m_mac_op_len     = m_mac_op_len_r    ;
assign m_mac_axis_data  = m_mac_axis_data_r ;
assign m_mac_axis_valid = m_mac_axis_valid_r;
assign m_mac_axis_last  = m_mac_axis_last_r ;

arbiter_rr #(
    .REQ_NUM (REQ_NUM)
) arbiter_rr_inst (
    .clk     (clk     ),
    .rst     (rst     ),
    .req     (req     ),
    .grant   (grant   ) 
);

endmodule