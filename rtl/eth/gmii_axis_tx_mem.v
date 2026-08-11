module gmii_axis_tx_mem (
    input        clk    ,
    input        rst    ,
    input  [7:0] s_data ,
    input        s_valid,
    input        s_last ,
    output       s_ready,
    output [7:0] m_data ,
    output       m_valid,
    output       m_last 
);

localparam IDLE  = 3'b001;
localparam WRITE = 3'b010;
localparam READ  = 3'b100;

reg [2:0] curr_state;
reg [2:0] next_state;

reg [7:0] s_data_ff ; 
reg       s_valid_ff;
reg       s_last_ff ; 

wire [8:0]  wdata;
wire        wen  ;
reg  [10:0] waddr;

wire        ren  ;
reg  [10:0] raddr;
wire [8:0]  rdata;
reg         ren_ff  ;
reg         ren_2ff ;
reg  [7:0]  rdata_ff;
reg         r_last  ;

reg         s_ready_r;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_data_ff  <= 8'b0;
        s_valid_ff <= 1'b0;
        s_last_ff  <= 1'b0;
    end
    else begin
        s_data_ff  <= s_data ;
        s_valid_ff <= s_valid;
        s_last_ff  <= s_last ;
    end
end

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
            if(s_valid == 1'b1)begin
                next_state = WRITE;
            end
            else begin
                next_state = IDLE;
            end
        end 
        WRITE: begin
            if((s_valid_ff == 1'b1) && (s_last_ff == 1'b1))begin
                next_state = READ;
            end
            else begin
                next_state = WRITE;
            end
        end
        READ: begin
            if((ren_ff == 1'b1) && (rdata[8] == 1'b1))begin
                next_state = IDLE;
            end
            else begin
                next_state = READ;
            end
        end
        default: begin
            next_state = IDLE;
        end
    endcase
end

assign wen   = (curr_state == WRITE) && (s_valid_ff == 1'b1) ;
assign wdata = {s_last_ff, s_data_ff};

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        waddr <= 11'd0;
    end
    else begin
        if((curr_state == IDLE) || (curr_state == READ))begin
            waddr <= 11'd0;
        end
        else begin
            if(wen == 1'b1)begin
                waddr <= waddr + 11'd1;
            end
        end
    end
end

assign ren = (curr_state == READ);

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        ren_ff  <= 1'b0;
        ren_2ff <= 1'b0;
    end
    else begin
        ren_ff  <= ren;
        ren_2ff <= ren_ff;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        raddr <= 11'd0;
    end
    else begin
        if(ren == 1'b1)begin
            raddr <= raddr + 11'd1;
        end
        else begin
            raddr <= 11'd0;
        end
    end
end

always @(posedge clk ) begin
    rdata_ff <= rdata[7:0];
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        r_last <= 1'b0;
    end
    else begin
        if((ren_ff == 1'b1) && (rdata[8] == 1'b1))begin
            r_last <= 1'b1;
        end
        else begin
            r_last <= 1'b0;
        end
    end
end

assign m_data  = rdata_ff[7:0];
assign m_valid = ren_2ff  &  ren_ff   ;
assign m_last  = r_last       ;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        s_ready_r <= 1'b0;
    end
    else begin
        if((curr_state == IDLE))begin
            s_ready_r <= 1'b1;
        end
        else begin
            if((curr_state == WRITE) && (s_valid == 1'b1) && (s_last == 1'b1))begin
                s_ready_r <= 1'b0;
            end
        end
    end
end

assign s_ready = s_ready_r;

tpram #(
    .DP     (2048),
    .WD     (9   )
) tpram_inst (
    .clk    (clk   ),
    .wdata  (wdata ),
    .wen    (wen   ),
    .waddr  (waddr ),
    .ren    (ren   ),
    .raddr  (raddr ),
    .rdata  (rdata )

);
    
endmodule