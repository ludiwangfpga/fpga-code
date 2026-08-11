module arp_rx (
    input         clk              ,
    input         rst              ,

    input  [31:0] cfg_ip           ,

    input  [47:0] s_mac_src_mac    ,
    input  [47:0] s_mac_dst_mac    ,
    input  [15:0] s_mac_op_len     ,
    input  [ 7:0] s_mac_axis_data  ,
    input         s_mac_axis_valid ,
    input         s_mac_axis_last  ,
    input         s_mac_axis_error ,

    input         req_ready        ,
    output        req_arp          , 
    output [47:0] req_src_mac      ,
    output [47:0] req_dst_mac      ,
    output [31:0] req_src_ip       ,
    output [31:0] req_dst_ip       ,

    input         resp_ready       ,
    output        resp_arp         , 
    output [47:0] resp_src_mac     ,
    output [47:0] resp_dst_mac     ,
    output [31:0] resp_src_ip      ,
    output [31:0] resp_dst_ip       

);

localparam IDLE = 2'b01;
localparam DATA = 2'b10;

reg [1:0] curr_state;
reg [1:0] next_state;

reg [ 7:0] s_mac_axis_data_ff ;
reg        s_mac_axis_valid_ff;
reg        s_mac_axis_last_ff ;
reg        s_mac_axis_error_ff;

reg [5:0] cnt;
reg [15:0] op;
reg [47:0] src_mac;
reg [47:0] dst_mac;
reg [31:0] src_ip;
reg [31:0] dst_ip;

reg req;
reg resp;

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
            if(s_mac_axis_valid == 1'b1)begin
                next_state = DATA;
            end
            else begin
                next_state = IDLE;
            end
        end 
        DATA: begin
            if(((s_mac_axis_last_ff == 1'b1) && (s_mac_axis_valid_ff == 1'b1)) || (s_mac_axis_error_ff == 1'b1))begin
                next_state = IDLE;
            end
            else begin
                next_state = DATA;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end


always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_mac_axis_data_ff  <= 8'b0;
        s_mac_axis_valid_ff <= 1'b0;
        s_mac_axis_last_ff  <= 1'b0;
        s_mac_axis_error_ff <= 1'b0;
    end
    else begin
        s_mac_axis_data_ff  <= s_mac_axis_data ;
        s_mac_axis_valid_ff <= s_mac_axis_valid;
        s_mac_axis_last_ff  <= s_mac_axis_last ;
        s_mac_axis_error_ff <= s_mac_axis_error;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        cnt <= 6'd0;
    end
    else begin
        if(curr_state == DATA)begin
            cnt <= cnt + 6'd1;
        end
        else begin
            cnt <= 6'd0;
        end
    end
end


always @(posedge clk ) begin
    if((cnt > 6'd5) && (cnt <= 6'd7))begin
        op <= {op[7:0], s_mac_axis_data_ff};
    end
    else if(cnt == 6'd1)begin
        op <= 16'd0;
    end
end

always @(posedge clk ) begin
    if((cnt > 6'd7) && (cnt <= 6'd13))begin
        src_mac <= {src_mac[39:0], s_mac_axis_data_ff};
    end
    else if(cnt == 6'd1)begin
        src_mac <= 48'd0;
    end
end

always @(posedge clk ) begin
    if((cnt > 6'd13) && (cnt <= 6'd17))begin
        src_ip <= {src_ip[23:0], s_mac_axis_data_ff};
    end
    else if(cnt == 6'd1)begin
        src_ip <= 32'd0;
    end
end

always @(posedge clk ) begin
    if((cnt > 6'd17) && (cnt <= 6'd23))begin
        dst_mac <= {dst_mac[39:0], s_mac_axis_data_ff};
    end
    else if(cnt == 6'd1)begin
        dst_mac <= 48'd0;
    end
end

always @(posedge clk ) begin
    if((cnt > 6'd23) && (cnt <= 6'd27))begin
        dst_ip <= {dst_ip[23:0], s_mac_axis_data_ff};
    end
    else if(cnt == 6'd1)begin
        dst_ip <= 32'd0;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        req <= 1'b0;
    end
    else begin
        if((cnt == 6'd28) && (req == 1'b0) && (dst_ip == cfg_ip) && (op == 16'd1))begin
            req <= 1'b1;
        end
        else if((req_ready == 1'b1) || (cnt == 6'd1))begin
            req <= 1'b0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        resp <= 1'b0;
    end
    else begin
        if((cnt == 6'd28) && (resp == 1'b0) && (dst_ip == cfg_ip) && (op == 16'd2))begin
            resp <= 1'b1;
        end
        else if((resp_ready == 1'b1) || (cnt == 6'd1))begin
            resp <= 1'b0;
        end
    end
end

assign req_arp     = req    ;
assign req_src_mac = src_mac;
assign req_dst_mac = dst_mac;
assign req_src_ip  = src_ip ;
assign req_dst_ip  = dst_ip ;

assign resp_arp     = resp   ;
assign resp_src_mac = src_mac;
assign resp_dst_mac = dst_mac;
assign resp_src_ip  = src_ip ;
assign resp_dst_ip  = dst_ip ;

    
endmodule