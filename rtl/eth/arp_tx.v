module arp_tx (
    input         clk              ,
    input         rst              ,

    input  [31:0] cfg_ip           ,
    input  [47:0] cfg_mac          ,

    input         arp_tx_valid     ,
    output        arp_tx_ready     ,
    input  [31:0] arp_tx_ip        ,
    input  [47:0] arp_tx_mac       ,
    input  [ 1:0] arp_tx_op        ,

    output [47:0] m_mac_src_mac    ,
    output [47:0] m_mac_dst_mac    ,
    output [15:0] m_mac_op_len     ,
    output [ 7:0] m_mac_axis_data  ,
    output        m_mac_axis_valid ,
    input         m_mac_axis_ready ,
    output        m_mac_axis_last  ,
    input         m_mac_axis_grant ,
    output        m_mac_axis_req   

);

localparam IDLE = 5'b00001;
localparam REQ  = 5'b00010;
localparam REDY = 5'b00100;
localparam SEND = 5'b01000;
localparam LAST = 5'b10000; 

reg [4:0] curr_state;
reg [4:0] next_state;


reg [47:0] m_mac_src_mac_r;
reg [47:0] m_mac_dst_mac_r;
reg [15:0] m_mac_op_len_r ;

reg [5:0] cnt;
reg [7:0] axis_data ;
reg       axis_valid;
reg       axis_last ;

reg [55:0] pre_data;
reg [55:0] pre_data_reg;
reg [47:0] src_mac;
reg [47:0] src_mac_reg;
reg [47:0] dst_mac;
reg [47:0] dst_mac_reg;
reg [31:0] src_ip;
reg [31:0] src_ip_reg;
reg [31:0] dst_ip;
reg [31:0] dst_ip_reg;

reg arp_tx_ready_r;
reg req_r;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        curr_state <= IDLE;
    end
    else begin
        curr_state <= next_state;
    end
end

always @(*) begin
    case (curr_state)
        IDLE: begin
            if(arp_tx_valid == 1'b1)begin
                next_state = REQ;
            end
            else begin
                next_state = IDLE;
            end
        end 
        REQ: begin
            if((m_mac_axis_req == 1'b1) && (m_mac_axis_grant == 1'b1))begin
                next_state = REDY;
            end
            else begin
                next_state = REQ;
            end
        end
        REDY: begin
            if(m_mac_axis_ready == 1'b1)begin
                next_state = SEND;
            end
            else begin
                next_state = REDY;
            end
        end
        SEND: begin
            if(cnt == 6'd45)begin
                next_state = LAST;
            end
            else begin
                next_state = SEND;
            end
        end
        LAST: begin
            next_state = IDLE;
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_mac_src_mac_r <= 48'd0;
        m_mac_dst_mac_r <= 48'd0;
        m_mac_op_len_r  <= 16'd0;
    end
    else begin
        if((arp_tx_valid == 1'b1) && (arp_tx_ready == 1'b1))begin
            m_mac_src_mac_r <= cfg_mac;
            m_mac_dst_mac_r <= arp_tx_mac;
            m_mac_op_len_r  <= 16'h0806;
        end
    end
end

always @(posedge clk ) begin
    if((arp_tx_valid == 1'b1) && (arp_tx_ready == 1'b1))begin
        pre_data <= {8'h01, 8'h08, 8'h00, 8'h06, 8'h04, 8'h00, {6'b0, arp_tx_op}};
        src_mac  <= cfg_mac;
        dst_mac  <= arp_tx_mac;
        src_ip   <= cfg_ip;
        dst_ip   <= arp_tx_ip;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        cnt <= 5'd0;
    end
    else begin
        if(curr_state == SEND)begin
            cnt <= cnt + 5'd1;
        end
        else begin
            cnt <= 5'd0;
        end
    end
end

always @(posedge clk ) begin
    if(cnt == 5'd0)begin
        axis_data    <= 8'd0;
        pre_data_reg <= pre_data;
        src_mac_reg  <= src_mac;
        dst_mac_reg  <= dst_mac;
        src_ip_reg   <= src_ip;
        dst_ip_reg   <= dst_ip;
    end
    else if((cnt >= 5'd1) && (cnt < 5'd8))begin
        axis_data    <= pre_data_reg[55 -: 8];
        pre_data_reg <= pre_data_reg << 4'd8;
    end
    else if((cnt >= 5'd8) && (cnt < 5'd14))begin
        axis_data   <= src_mac_reg[47 -: 8];
        src_mac_reg <= src_mac_reg << 4'd8;
    end
    else if((cnt >= 5'd14) && (cnt < 5'd18))begin
        axis_data  <= src_ip_reg[31 -: 8];
        src_ip_reg <= src_ip_reg << 4'd8;
    end
    else if((cnt >= 5'd18) && (cnt < 5'd24))begin
        axis_data   <= dst_mac_reg[47 -: 8];
        dst_mac_reg <= dst_mac_reg << 4'd8;
    end
    else if((cnt >= 5'd24) && (cnt < 5'd28))begin
        axis_data  <= dst_ip_reg[31 -: 8];
        dst_ip_reg <= dst_ip_reg << 4'd8;
    end
    else begin
        axis_data    <= 8'd0;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        axis_valid <= 1'b0;
    end
    else begin
        if(curr_state == SEND)begin
            axis_valid <= 1'b1;
        end
        else begin
            axis_valid <= 1'b0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        axis_last <= 1'b0;
    end
    else begin
        axis_last <= cnt == 6'd45;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        arp_tx_ready_r <= 1'b0;
    end
    else begin
        if(curr_state == IDLE)begin
            arp_tx_ready_r <= 1'b1;
        end
        else begin
            arp_tx_ready_r <= 1'b0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        req_r <= 1'b0;
    end
    else begin
        if((curr_state == IDLE) && (next_state == REQ))begin
            req_r <= 1'b1;
        end
        else begin
            if((req_r == 1'b1) && (m_mac_axis_grant == 1'b1))begin
                req_r <= 1'b0;
            end
        end
    end
end

assign m_mac_src_mac    = m_mac_src_mac_r;
assign m_mac_dst_mac    = m_mac_dst_mac_r;
assign m_mac_op_len     = m_mac_op_len_r ;
assign m_mac_axis_data  = axis_data ;
assign m_mac_axis_valid = axis_valid;
assign m_mac_axis_last  = axis_last ;

assign arp_tx_ready = arp_tx_ready_r;
assign m_mac_axis_req = req_r;
    
endmodule