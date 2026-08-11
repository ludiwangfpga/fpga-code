module gmii_axis_tx #(
    parameter DEVICE     = "XILINX"
) (
    input        clk              ,
    input        rst              ,

    input  [7:0] s_eth_axis_data  ,
    input        s_eth_axis_valid ,
    input        s_eth_axis_last  ,
    output       s_eth_axis_ready ,

    input        mac_tx_gmii_clk  ,
    output [7:0] mac_tx_gmii_data ,
    output       mac_tx_gmii_dv     
);

// reg       s_mac_tx_gmii_clk ;
// reg [7:0] s_mac_tx_gmii_data;
// reg       s_mac_tx_gmii_dv  ;
// reg       s_mac_tx_gmii_err ;

wire [7:0] fifo_axis_data ;
wire       fifo_axis_valid;
wire       fifo_axis_last ;
wire       fifo_axis_ready;

reg rst_r;
reg rst_2r;

reg w_ping;
reg r_ping;

wire [7:0] s_data    ;
wire       s_valid   ;
wire       s_last    ;
wire       s_ready   ;

wire [7:0] m_data    ;
wire       m_valid   ;
wire       m_last    ;

wire [7:0] ping_s_data ;
wire       ping_s_valid;
wire       ping_s_last ;
wire       ping_s_ready;

wire [7:0] pang_s_data ;
wire       pang_s_valid;
wire       pang_s_last ;
wire       pang_s_ready;

wire [7:0] ping_m_data ;
wire       ping_m_valid;
wire       ping_m_last ;

wire [7:0] pang_m_data ;
wire       pang_m_valid;
wire       pang_m_last ;

reg  [7:0] m_data_ff ;
reg        m_valid_ff;

localparam SEND = 2'b01;
localparam GAP  = 2'b10;

localparam GAP_CNT = 7'd96;
reg [1:0] curr_state;
reg [1:0] next_state;

reg [6:0] gap_cnt;

wire [8:0] dout;
wire       full;
wire       empty;
wire       rd_en;
wire       wr_en;
wire [8:0] din;
wire       wr_rst_busy;
wire       rd_rst_busy;

always @(posedge mac_tx_gmii_clk ) begin
    rst_r  <= rst  ;
    rst_2r <= rst_r;
end

always @(posedge mac_tx_gmii_clk ) begin
    if(rst_2r == 1'b1)begin
        w_ping <= 1'b0;
    end
    else begin
        if((s_valid == 1'b1) && (s_ready == 1'b1) && (s_last == 1'b1))begin
            w_ping <= !w_ping;
        end
    end
end

always @(posedge mac_tx_gmii_clk ) begin
    if(rst_2r == 1'b1)begin
        r_ping <= 1'b0;
    end
    else begin
        if((m_valid == 1'b1) && (m_last == 1'b1))begin
            r_ping <= !r_ping;
        end
    end
end

assign ping_s_data  = (w_ping == 1'b0) ? dout[7:0]  : 8'b0;
assign ping_s_valid = (w_ping == 1'b0) ? s_valid    : 1'b0;
assign ping_s_last  = (w_ping == 1'b0) ? dout[8]    : 1'b0;

assign pang_s_data  = (w_ping == 1'b1) ? dout[7:0]  : 8'b0;
assign pang_s_valid = (w_ping == 1'b1) ? s_valid    : 1'b0;
assign pang_s_last  = (w_ping == 1'b1) ? dout[8]    : 1'b0;

assign s_ready = (w_ping == 1'b0) ? ping_s_ready : pang_s_ready;

assign m_valid = (r_ping == 1'b0) ? ping_m_valid : pang_m_valid;
assign m_data  = (r_ping == 1'b0) ? ping_m_data  : pang_m_data ;
assign m_last  = (r_ping == 1'b0) ? ping_m_last  : pang_m_last ;

always @(posedge mac_tx_gmii_clk ) begin
    if(rst_2r == 1'b1)begin
        m_data_ff  <= 8'b0;
        m_valid_ff <= 1'b0;
    end
    else begin
        m_data_ff  <= m_data ;
        m_valid_ff <= m_valid;
    end
end

assign mac_tx_gmii_data = m_data_ff ;
assign mac_tx_gmii_dv   = m_valid_ff; 

gmii_axis_tx_mem gmii_axis_tx_mem_ping(
    .clk        (mac_tx_gmii_clk),
    .rst        (rst_2r         ),
    .s_data     (ping_s_data    ),
    .s_valid    (ping_s_valid   ),
    .s_last     (ping_s_last    ),
    .s_ready    (ping_s_ready   ),
    .m_data     (ping_m_data    ),
    .m_valid    (ping_m_valid   ),
    .m_last     (ping_m_last    )
);

gmii_axis_tx_mem gmii_axis_tx_mem_pang(
    .clk        (mac_tx_gmii_clk),
    .rst        (rst_2r         ),
    .s_data     (pang_s_data    ),
    .s_valid    (pang_s_valid   ),
    .s_last     (pang_s_last    ),
    .s_ready    (pang_s_ready   ),
    .m_data     (pang_m_data    ),
    .m_valid    (pang_m_valid   ),
    .m_last     (pang_m_last    )
);

stream_fifo #(
    .DP (2048),
    .WD (8   )
) stream_fifo_inst (
    .clk     (clk              ),
    .rst     (rst              ),
    .s_data  (s_eth_axis_data  ),
    .s_valid (s_eth_axis_valid ),
    .s_last  (s_eth_axis_last  ),
    .s_ready (s_eth_axis_ready ),

    .m_data  (fifo_axis_data   ),
    .m_valid (fifo_axis_valid  ),
    .m_last  (fifo_axis_last   ),
    .m_ready (fifo_axis_ready  ) 
);


always @(posedge clk ) begin
    if(rst == 1'b1)begin
        curr_state <= SEND;
    end
    else begin
        curr_state <= next_state;
    end
end

always @(*) begin
    case (curr_state)
        SEND: begin
            if((fifo_axis_last == 1'b1) && (fifo_axis_ready == 1'b1) && (fifo_axis_valid == 1'b1))begin
                next_state = GAP;
            end
            else begin
                next_state = SEND;
            end
        end 
        GAP: begin
            if(gap_cnt == GAP_CNT)begin
                next_state = SEND;
            end
            else begin
                next_state = GAP;
            end
        end
        default: begin
            next_state = SEND;
        end
    endcase
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        gap_cnt <= 7'd0;
    end
    else begin
        if(curr_state == GAP)begin
            gap_cnt <= gap_cnt + 7'd1;
        end
    end
end

assign s_valid = rd_en;
assign s_last  = dout[8];
assign rd_en   = (empty == 1'b0) && (rd_rst_busy == 1'b0) && (s_ready == 1'b1);
assign wr_en   = (fifo_axis_ready == 1'b1) && (fifo_axis_valid == 1'b1);
assign fifo_axis_ready = (full == 1'b0) && (wr_rst_busy == 1'b0) && (curr_state == SEND);
assign din     = {fifo_axis_last, fifo_axis_data};

generate
    if(DEVICE == "XILINX")begin
   xpm_fifo_async #(
      .CASCADE_HEIGHT(0),        // DECIMAL
      .CDC_SYNC_STAGES(2),       // DECIMAL
      .DOUT_RESET_VALUE("0"),    // String
      .ECC_MODE("no_ecc"),       // String
      .FIFO_MEMORY_TYPE("auto"), // String
      .FIFO_READ_LATENCY(1),     // DECIMAL
      .FIFO_WRITE_DEPTH(512),   // DECIMAL
      .FULL_RESET_VALUE(0),      // DECIMAL
      .PROG_EMPTY_THRESH(10),    // DECIMAL
      .PROG_FULL_THRESH(10),     // DECIMAL
      .RD_DATA_COUNT_WIDTH(1),   // DECIMAL
      .READ_DATA_WIDTH(9),      // DECIMAL
      .READ_MODE("fwft"),         // String
      .RELATED_CLOCKS(0),        // DECIMAL
      .SIM_ASSERT_CHK(0),        // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
      .USE_ADV_FEATURES("0707"), // String
      .WAKEUP_TIME(0),           // DECIMAL
      .WRITE_DATA_WIDTH(9),     // DECIMAL
      .WR_DATA_COUNT_WIDTH(9)    // DECIMAL
   )
   xpm_fifo_async_inst (
      .almost_empty(),   // 1-bit output: Almost Empty : When asserted, this signal indicates that
                                     // only one more read can be performed before the FIFO goes to empty.

      .almost_full(),     // 1-bit output: Almost Full: When asserted, this signal indicates that
                                     // only one more write can be performed before the FIFO is full.

      .data_valid(),       // 1-bit output: Read Data Valid: When asserted, this signal indicates
                                     // that valid data is available on the output bus (dout).

      .dbiterr(),             // 1-bit output: Double Bit Error: Indicates that the ECC decoder detected
                                     // a double-bit error and data in the FIFO core is corrupted.

      .dout(dout),                   // READ_DATA_WIDTH-bit output: Read Data: The output data bus is driven
                                     // when reading the FIFO.

      .empty(empty),                 // 1-bit output: Empty Flag: When asserted, this signal indicates that the
                                     // FIFO is empty. Read requests are ignored when the FIFO is empty,
                                     // initiating a read while empty is not destructive to the FIFO.

      .full(full),                   // 1-bit output: Full Flag: When asserted, this signal indicates that the
                                     // FIFO is full. Write requests are ignored when the FIFO is full,
                                     // initiating a write when the FIFO is full is not destructive to the
                                     // contents of the FIFO.

      .overflow(),           // 1-bit output: Overflow: This signal indicates that a write request
                                     // (wren) during the prior clock cycle was rejected, because the FIFO is
                                     // full. Overflowing the FIFO is not destructive to the contents of the
                                     // FIFO.

      .prog_empty(),       // 1-bit output: Programmable Empty: This signal is asserted when the
                                     // number of words in the FIFO is less than or equal to the programmable
                                     // empty threshold value. It is de-asserted when the number of words in
                                     // the FIFO exceeds the programmable empty threshold value.

      .prog_full(),         // 1-bit output: Programmable Full: This signal is asserted when the
                                     // number of words in the FIFO is greater than or equal to the
                                     // programmable full threshold value. It is de-asserted when the number of
                                     // words in the FIFO is less than the programmable full threshold value.

      .rd_data_count(), // RD_DATA_COUNT_WIDTH-bit output: Read Data Count: This bus indicates the
                                     // number of words read from the FIFO.

      .rd_rst_busy(rd_rst_busy),     // 1-bit output: Read Reset Busy: Active-High indicator that the FIFO read
                                     // domain is currently in a reset state.

      .sbiterr(),             // 1-bit output: Single Bit Error: Indicates that the ECC decoder detected
                                     // and fixed a single-bit error.

      .underflow(),         // 1-bit output: Underflow: Indicates that the read request (rd_en) during
                                     // the previous clock cycle was rejected because the FIFO is empty. Under
                                     // flowing the FIFO is not destructive to the FIFO.

      .wr_ack(),               // 1-bit output: Write Acknowledge: This signal indicates that a write
                                     // request (wr_en) during the prior clock cycle is succeeded.

      .wr_data_count(), // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus indicates
                                     // the number of words written into the FIFO.

      .wr_rst_busy(wr_rst_busy),     // 1-bit output: Write Reset Busy: Active-High indicator that the FIFO
                                     // write domain is currently in a reset state.

      .din(din),                     // WRITE_DATA_WIDTH-bit input: Write Data: The input data bus used when
                                     // writing the FIFO.

      .injectdbiterr(1'b0), // 1-bit input: Double Bit Error Injection: Injects a double bit error if
                                     // the ECC feature is used on block RAMs or UltraRAM macros.

      .injectsbiterr(1'b0), // 1-bit input: Single Bit Error Injection: Injects a single bit error if
                                     // the ECC feature is used on block RAMs or UltraRAM macros.

      .rd_clk(mac_tx_gmii_clk),               // 1-bit input: Read clock: Used for read operation. rd_clk must be a free
                                     // running clock.

      .rd_en(rd_en),                 // 1-bit input: Read Enable: If the FIFO is not empty, asserting this
                                     // signal causes data (on dout) to be read from the FIFO. Must be held
                                     // active-low when rd_rst_busy is active high.

      .rst(rst),                     // 1-bit input: Reset: Must be synchronous to wr_clk. The clock(s) can be
                                     // unstable at the time of applying reset, but reset must be released only
                                     // after the clock(s) is/are stable.

      .sleep(1'b0),                 // 1-bit input: Dynamic power saving: If sleep is High, the memory/fifo
                                     // block is in power saving mode.

      .wr_clk(clk),               // 1-bit input: Write clock: Used for write operation. wr_clk must be a
                                     // free running clock.

      .wr_en(wr_en)                  // 1-bit input: Write Enable: If the FIFO is not full, asserting this
                                     // signal causes data (on din) to be written to the FIFO. Must be held
                                     // active-low when rst or wr_rst_busy is active high.

   );
        end
endgenerate
endmodule