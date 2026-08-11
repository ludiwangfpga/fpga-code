module udp_rx (
    input         clk              ,
    input         rst              ,

    input  [15:0] cfg_udp_src_port ,
    input  [15:0] cfg_udp_dst_port ,

    input  [ 7:0] s_ip_axis_data   ,
    input         s_ip_axis_valid  ,
    input         s_ip_axis_last   ,
    input         s_ip_axis_error  ,

    output [ 7:0] m_udp_axis_data  ,
    output        m_udp_axis_valid ,
    output        m_udp_axis_last  ,
    output        m_udp_axis_error   
);

reg [11:0] cnt;

reg [15:0] src_port;
reg [15:0] dst_port;

reg valid_flag;

reg [7:0] m_udp_axis_data_r ;
reg       m_udp_axis_valid_r;
reg       m_udp_axis_last_r ;
reg       m_udp_axis_error_r;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        cnt <= 12'd0;
    end
    else begin
        if(s_ip_axis_valid == 1'b1)begin
            cnt <= cnt + 12'd1;
        end
        else begin
            cnt <= 12'd0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        src_port <= 16'd0;
    end
    else begin
        if(s_ip_axis_valid == 1'b1)begin
            if(cnt == 12'd0)begin
                src_port[15:8] <= s_ip_axis_data;
            end
            else if(cnt == 12'd1)begin
                src_port[7:0] <= s_ip_axis_data;
            end 
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        dst_port <= 16'd0;
    end
    else begin
        if(s_ip_axis_valid == 1'b1)begin
            if(cnt == 12'd2)begin
                dst_port[15:8] <= s_ip_axis_data;
            end
            else if(cnt == 12'd3)begin
                dst_port[7:0] <= s_ip_axis_data;
            end 
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        valid_flag <= 1'b0;
    end
    else begin
        if(s_ip_axis_valid == 1'b1)begin
            if(cnt == 12'd4)begin
                if(dst_port == cfg_udp_src_port)begin
                    valid_flag <= 1'b1;
                end
            end
        end
        else begin
            valid_flag <= 1'b0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_udp_axis_data_r  <= 8'd0;
        m_udp_axis_valid_r <= 1'd0;
        m_udp_axis_last_r  <= 1'd0;
        m_udp_axis_error_r <= 1'd0;
    end
    else begin
        if((valid_flag == 1'b1) && (s_ip_axis_valid == 1'b1) && (cnt >= 12'd8))begin
            m_udp_axis_data_r  <= s_ip_axis_data ;
            m_udp_axis_valid_r <= s_ip_axis_valid;
            m_udp_axis_last_r  <= s_ip_axis_last ;
            m_udp_axis_error_r <= s_ip_axis_error;
        end
        else begin
            m_udp_axis_data_r  <= 8'd0;
            m_udp_axis_valid_r <= 1'd0;
            m_udp_axis_last_r  <= 1'd0;
            m_udp_axis_error_r <= 1'd0;
        end
    end
end

assign m_udp_axis_data  = m_udp_axis_data_r ;
assign m_udp_axis_valid = m_udp_axis_valid_r;
assign m_udp_axis_last  = m_udp_axis_last_r ;
assign m_udp_axis_error = m_udp_axis_error_r;
    
endmodule