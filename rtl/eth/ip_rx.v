module ip_rx (
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

    output [ 7:0] m_ip_protocol    ,
    output [ 7:0] m_ip_axis_data   ,
    output        m_ip_axis_valid  ,
    output        m_ip_axis_last   ,
    output        m_ip_axis_error   
);
  
reg [ 7:0] s_mac_axis_data_ff ;
reg        s_mac_axis_valid_ff;
reg        s_mac_axis_last_ff ;
reg        s_mac_axis_error_ff;


reg [11:0] cnt;
reg [ 7:0] protocol;
reg [31:0] dst_ip;
reg        valid_flag;

reg [ 7:0] m_ip_protocol_r  ;  
reg [ 7:0] m_ip_axis_data_r ; 
reg        m_ip_axis_valid_r;
reg        m_ip_axis_last_r ; 
reg        m_ip_axis_error_r;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_mac_axis_data_ff  <= 8'd0;
        s_mac_axis_valid_ff <= 1'd0;
        s_mac_axis_last_ff  <= 1'd0;
        s_mac_axis_error_ff <= 1'd0;
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
        cnt <= 12'd0;
    end
    else begin
        if(s_mac_axis_valid == 1'b1)begin
            cnt <= cnt + 12'd1;
        end
        else begin
            cnt <= 12'd0;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        protocol <= 8'd0;
    end
    else begin
        if((s_mac_axis_valid == 1'b1) && (cnt == 12'd9))begin
            protocol <= s_mac_axis_data;
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        dst_ip <= 32'd0;
    end
    else begin
        if((s_mac_axis_valid == 1'b1) && ((cnt >= 12'd16) && (cnt <= 12'd19)))begin
            dst_ip <= {dst_ip[24:0], s_mac_axis_data};
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        valid_flag <= 1'b0;
    end
    else begin
        if((cnt == 12'd20) && (dst_ip == cfg_ip))begin
            valid_flag <= 1'b1;
        end
        else begin
            if((s_mac_axis_valid_ff == 1'b0) || ((s_mac_axis_valid_ff == 1'b1) && (s_mac_axis_last_ff == 1'b1)))begin
                valid_flag <= 1'b0;
            end
        end
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_ip_protocol_r   <= 8'b0;
        m_ip_axis_data_r  <= 8'b0;
        m_ip_axis_valid_r <= 1'b0;
        m_ip_axis_last_r  <= 1'b0;
        m_ip_axis_error_r <= 1'b0;
    end
    else begin
        if(valid_flag == 1'b1)begin
            m_ip_protocol_r   <= protocol            ;
            m_ip_axis_data_r  <= s_mac_axis_data_ff ;
            m_ip_axis_valid_r <= s_mac_axis_valid_ff;
            m_ip_axis_last_r  <= s_mac_axis_last_ff ;
            m_ip_axis_error_r <= s_mac_axis_error_ff;
        end
        else begin
            m_ip_protocol_r   <= 8'b0;
            m_ip_axis_data_r  <= 8'b0;
            m_ip_axis_valid_r <= 1'b0;
            m_ip_axis_last_r  <= 1'b0;
            m_ip_axis_error_r <= 1'b0;
        end
    end
end


assign m_ip_protocol   = m_ip_protocol_r  ;
assign m_ip_axis_data  = m_ip_axis_data_r ;
assign m_ip_axis_valid = m_ip_axis_valid_r;
assign m_ip_axis_last  = m_ip_axis_last_r ;
assign m_ip_axis_error = m_ip_axis_error_r;

    
endmodule