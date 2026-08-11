module eth_tx_ctrl (
    input         clk                  ,
    input         rst                  ,
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

localparam REQ_NUM = 2;

localparam IDLE  = 4'b0001;
localparam GRANT = 4'b0010;
localparam ARP   = 4'b0100;
localparam IP    = 4'b1000;

reg [3:0] curr_state;
reg [3:0] next_state;

wire [REQ_NUM - 1:0] grant    ;
wire [REQ_NUM - 1:0] req      ;
reg  [REQ_NUM - 1:0] req_ff   ;
reg  [REQ_NUM - 1:0] grant_ff ;

reg s_arp_mac_axis_ready_r;
reg s_ip_mac_axis_ready_r;

reg [47:0] m_mac_src_mac_r    ;
reg [47:0] m_mac_dst_mac_r    ;
reg [15:0] m_mac_op_len_r     ;
reg [ 7:0] m_mac_axis_data_r  ;
reg        m_mac_axis_valid_r ;
reg        m_mac_axis_last_r  ;
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
            if(|req)begin
                next_state = GRANT;
            end
            else begin
                next_state = IDLE;
            end
        end
        GRANT: begin
            if(grant_ff[0] == 1'b1)begin
                next_state = ARP;
            end
            else if(grant_ff[1] == 1'b1)begin
                next_state = IP;
            end
            else begin
                next_state = GRANT;
            end
        end
        ARP: begin
            if((s_arp_mac_axis_last == 1'b1) && (s_arp_mac_axis_valid == 1'b1))begin
                next_state = IDLE;
            end
            else begin
                next_state = ARP;
            end
        end
        IP: begin
            if((s_ip_mac_axis_last == 1'b1) && (s_ip_mac_axis_valid == 1'b1))begin
                next_state = IDLE;
            end
            else begin
                next_state = IP;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        req_ff   <= {REQ_NUM{1'b0}};
        grant_ff <= {REQ_NUM{1'b0}}; 
    end
    else begin
        req_ff   <= req  ;
        grant_ff <= grant;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_arp_mac_axis_ready_r <= 1'b0;
    end
    else begin
        if(curr_state == ARP)begin
            s_arp_mac_axis_ready_r <= m_mac_axis_ready;
        end
        else begin
            s_arp_mac_axis_ready_r <= 1'b0;
        end
    end
end

assign s_arp_mac_axis_ready = s_arp_mac_axis_ready_r & m_mac_axis_ready;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ip_mac_axis_ready_r <= 1'b0;
    end
    else begin
        if(curr_state == IP)begin
            s_ip_mac_axis_ready_r <= m_mac_axis_ready;
        end
        else begin
            s_ip_mac_axis_ready_r <= 1'b0;
        end
    end
end

assign s_ip_mac_axis_ready = s_ip_mac_axis_ready_r & m_mac_axis_ready;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_mac_src_mac_r    <= 48'd0;
        m_mac_dst_mac_r    <= 48'd0;
        m_mac_op_len_r     <= 16'd0;
        m_mac_axis_data_r  <=  8'd0;
        m_mac_axis_valid_r <=  1'b0;
        m_mac_axis_last_r  <=  1'b0;
    end 
    else begin
        if(curr_state == ARP)begin
            m_mac_src_mac_r    <= s_arp_mac_src_mac   ;
            m_mac_dst_mac_r    <= s_arp_mac_dst_mac   ;
            m_mac_op_len_r     <= s_arp_mac_op_len    ;
            m_mac_axis_data_r  <= s_arp_mac_axis_data ;
            m_mac_axis_valid_r <= s_arp_mac_axis_valid;
            m_mac_axis_last_r  <= s_arp_mac_axis_last ;
        end
        else if(curr_state == IP)begin
            m_mac_src_mac_r    <= s_ip_mac_src_mac    ;
            m_mac_dst_mac_r    <= s_ip_mac_dst_mac    ;
            m_mac_op_len_r     <= s_ip_mac_op_len     ;
            m_mac_axis_data_r  <= s_ip_mac_axis_data  ;
            m_mac_axis_valid_r <= s_ip_mac_axis_valid ;
            m_mac_axis_last_r  <= s_ip_mac_axis_last  ;
        end
        else begin
            m_mac_src_mac_r    <= 48'd0;
            m_mac_dst_mac_r    <= 48'd0;
            m_mac_op_len_r     <= 16'd0;
            m_mac_axis_data_r  <=  8'd0;
            m_mac_axis_valid_r <=  1'b0;
            m_mac_axis_last_r  <=  1'b0;
        end
    end
end

assign req = (curr_state == IDLE) ? {s_ip_mac_axis_req, s_arp_mac_axis_req} : {REQ_NUM{1'b0}};
assign s_arp_mac_axis_grant = grant_ff[0];
assign s_ip_mac_axis_grant  = grant_ff[1];

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
    .req     (req_ff  ),
    .grant   (grant) 
);

    
endmodule