module eth_mac_rx (
    input         clk              ,
    input         rst              ,

    input [47:0]  cfg_mac          ,

    input  [ 7:0] s_eth_axis_data  ,
    input         s_eth_axis_valid ,
    input         s_eth_axis_last  ,
    input         s_eth_axis_error ,
    output [47:0] m_mac_src_mac    ,
    output [47:0] m_mac_dst_mac    ,
    output [15:0] m_mac_op_len     ,     
    output [ 7:0] m_mac_axis_data  ,
    output        m_mac_axis_valid ,
    output        m_mac_axis_last  ,
    output        m_mac_axis_error  
);

localparam IDLE = 4'b0001;
localparam PRE  = 4'b0010;
localparam DATA = 4'b0100;
localparam WAIT = 4'b1000;

reg [47:0] src_mac;
reg [47:0] dst_mac;
reg [15:0] op_len ;
reg [ 7:0] data;
reg        valid;
reg        last;
reg        error;

reg [3:0] curr_state;
reg [3:0] next_state;

reg [7:0] s_eth_axis_data_ff [3:0] ;

reg [10:0] cnt;

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
            if((s_eth_axis_valid == 1'b1) && (s_eth_axis_data == 8'h55))begin
                next_state = PRE;
            end
            else begin
                next_state = IDLE;
            end
        end 
        PRE: begin
            if(s_eth_axis_valid == 11'b0)begin
                next_state = IDLE;
            end
            else begin
                if((cnt < 11'd6) && (s_eth_axis_data != 8'h55))begin
                    next_state = IDLE;
                end
                else if((cnt == 11'd6) && (s_eth_axis_data == 8'hd5))begin
                    next_state = DATA;
                end
                else begin
                    next_state = PRE;
                end
            end
        end
        DATA: begin
            if(((cnt == 11'd13) && (dst_mac != cfg_mac) && (dst_mac != 48'hff_ff_ff_ff_ff_ff)))begin
                next_state = IDLE;
            end
            else if(((s_eth_axis_last == 1'b1) && (s_eth_axis_valid == 1'b1)))begin
                next_state = WAIT;
            end
            else begin
                next_state = DATA;
            end
        end
        WAIT: begin
            if(m_mac_axis_last ==1'b1)begin
                next_state = IDLE;
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

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        cnt <= 11'd0;
    end
    else begin
        if(curr_state == IDLE)begin
            cnt <= 11'd0;
        end
        else begin
            cnt <= cnt + 11'd1;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        dst_mac <= 48'd0;
    end
    else begin
        if(curr_state == IDLE)begin
            dst_mac <= 48'd0;
        end
        else if((cnt > 11'd6) && (cnt <= 11'd12))begin
            dst_mac <= {dst_mac[39:0], s_eth_axis_data};
        end
    end
end
    
always @(posedge clk ) begin
    if(rst == 1'b1)begin
        src_mac <= 48'd0;
    end
    else begin
        if(curr_state == IDLE)begin
            src_mac <= 48'd0;
        end
        else if((cnt > 11'd12) && (cnt <= 11'd18))begin
            src_mac <= {src_mac[39:0], s_eth_axis_data};
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        op_len <= 16'd0;
    end
    else begin
        if(curr_state == IDLE)begin
            op_len <= 16'd0;
        end
        else if((cnt > 11'd18) && (cnt <= 11'd20))begin
            op_len <= {op_len[7:0], s_eth_axis_data};
        end
    end
end

always @(posedge clk) begin
    s_eth_axis_data_ff[0] <= s_eth_axis_data;
end

genvar i;
generate
    for(i = 0; i < 3; i = i + 1)begin : DATA_FF_LOOP
        always @(posedge clk ) begin
            s_eth_axis_data_ff[i+1] <= s_eth_axis_data_ff[i];
        end
    end
endgenerate

always @(posedge clk ) begin
    data <= s_eth_axis_data_ff[3];
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        valid <= 1'b0;
    end
    else begin
        if(((curr_state == DATA) && (next_state == IDLE)) || (curr_state == IDLE))begin
            valid <= 1'b0;
        end
        else if(cnt > 11'd24)begin
            valid <= s_eth_axis_valid;  
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        last <= 1'b0;
    end
    else begin
        last <= (curr_state == DATA) && (next_state == WAIT);
    end
end

always @(posedge clk) begin
    if(rst == 1'b1)begin
        error <= 1'b0;
    end
    else begin
        error <= s_eth_axis_error;
    end
end

assign m_mac_dst_mac = dst_mac;
assign m_mac_src_mac = src_mac;
assign m_mac_op_len  = op_len ;

assign m_mac_axis_data  = data ;
assign m_mac_axis_valid = valid;
assign m_mac_axis_last  = last ;
assign m_mac_axis_error = error; 

endmodule