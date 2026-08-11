module gmii_axis_rx #(
    parameter DEVICE     = "XILINX"
) (
    input            clk              ,
    input            rst              ,
    input            mac_rx_gmii_clk  ,
    input      [7:0] mac_rx_gmii_data ,
    input            mac_rx_gmii_dv   ,
    input            mac_rx_gmii_err  ,    
    output reg [7:0] m_eth_axis_data  ,
    output reg       m_eth_axis_valid ,
    output reg       m_eth_axis_last  ,
    output reg       m_eth_axis_error 
);

reg [7:0] mac_rx_gmii_data_ff; 
reg       mac_rx_gmii_dv_ff  ;  
reg       mac_rx_gmii_err_ff ; 

wire [7:0] s_axis_tdata ;
wire       s_axis_tvalid;
wire       s_axis_tready;
wire [3:0] s_axis_tuser ;
wire       s_axis_tlast ;

wire [7:0] m_axis_tdata ;
wire       m_axis_tvalid;
wire       m_axis_tready;
wire [3:0] m_axis_tuser ;
wire       m_axis_tlast ;

always @(posedge mac_rx_gmii_clk ) begin
    mac_rx_gmii_data_ff <= mac_rx_gmii_data;
    mac_rx_gmii_dv_ff   <= mac_rx_gmii_dv  ;
    mac_rx_gmii_err_ff  <= mac_rx_gmii_err ;
end

assign s_axis_tvalid = mac_rx_gmii_dv_ff  ;
assign s_axis_tdata  = mac_rx_gmii_data_ff;
assign s_axis_tuser  = mac_rx_gmii_err_ff ;
assign s_axis_tlast  = mac_rx_gmii_dv_ff & (!mac_rx_gmii_dv);
assign m_axis_tready = 1'b1;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        m_eth_axis_data  <= 7'b0;
        m_eth_axis_valid <= 1'b0;
        m_eth_axis_last  <= 1'b0;
        m_eth_axis_error <= 1'b0;
    end
    else begin
        m_eth_axis_data  <= m_axis_tdata ;
        m_eth_axis_valid <= m_axis_tvalid;
        m_eth_axis_last  <= m_axis_tlast ;
        m_eth_axis_error <= |m_axis_tuser;
    end
end

generate
    if(DEVICE == "XILINX")begin


        xpm_fifo_axis #(
            .CASCADE_HEIGHT(0),             // DECIMAL
            .CDC_SYNC_STAGES(2),            // DECIMAL
            .CLOCKING_MODE("independent_clock"), // String
            .ECC_MODE("no_ecc"),            // String
            .FIFO_DEPTH(4096),              // DECIMAL
            .FIFO_MEMORY_TYPE("block"),      // String
            .PACKET_FIFO("true"),          // String
            .PROG_EMPTY_THRESH(10),         // DECIMAL
            .PROG_FULL_THRESH(10),          // DECIMAL
            .RD_DATA_COUNT_WIDTH(8),        // DECIMAL
            .RELATED_CLOCKS(0),             // DECIMAL
            .SIM_ASSERT_CHK(0),             // DECIMAL; 0=disable simulation messages, 1=enable simulation messages
            .TDATA_WIDTH(8),               // DECIMAL
            .TDEST_WIDTH(1),                // DECIMAL
            .TID_WIDTH(1),                  // DECIMAL
            .TUSER_WIDTH(4),                // DECIMAL
            .USE_ADV_FEATURES("0000"),      // String
            .WR_DATA_COUNT_WIDTH(8)         // DECIMAL
       ) xpm_fifo_axis_inst (
            .almost_empty_axis(),   // 1-bit output: Almost Empty : When asserted, this signal
                                                     // indicates that only one more read can be performed before the
                                                     // FIFO goes to empty.
      
            .almost_full_axis(),     // 1-bit output: Almost Full: When asserted, this signal
                                                     // indicates that only one more write can be performed before
                                                     // the FIFO is full.
      
            .dbiterr_axis(),             // 1-bit output: Double Bit Error- Indicates that the ECC
                                                     // decoder detected a double-bit error and data in the FIFO core
                                                     // is corrupted.
      
            .m_axis_tdata(m_axis_tdata),             // TDATA_WIDTH-bit output: TDATA: The primary payload that is
                                                     // used to provide the data that is passing across the
                                                     // interface. The width of the data payload is an integer number
                                                     // of bytes.
      
            .m_axis_tdest(),             // TDEST_WIDTH-bit output: TDEST: Provides routing information
                                                     // for the data stream.
      
            .m_axis_tid(),                 // TID_WIDTH-bit output: TID: The data stream identifier that
                                                     // indicates different streams of data.
      
            .m_axis_tkeep(),             // TDATA_WIDTH/8-bit output: TKEEP: The byte qualifier that
                                                     // indicates whether the content of the associated byte of TDATA
                                                     // is processed as part of the data stream. Associated bytes
                                                     // that have the TKEEP byte qualifier deasserted are null bytes
                                                     // and can be removed from the data stream. For a 64-bit DATA,
                                                     // bit 0 corresponds to the least significant byte on DATA, and
                                                     // bit 7 corresponds to the most significant byte. For example:
                                                     // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
                                                     // DATA[63:56] is a NULL byte
      
            .m_axis_tlast(m_axis_tlast),             // 1-bit output: TLAST: Indicates the boundary of a packet.
            .m_axis_tstrb(),             // TDATA_WIDTH/8-bit output: TSTRB: The byte qualifier that
                                                     // indicates whether the content of the associated byte of TDATA
                                                     // is processed as a data byte or a position byte. For a 64-bit
                                                     // DATA, bit 0 corresponds to the least significant byte on
                                                     // DATA, and bit 0 corresponds to the least significant byte on
                                                     // DATA, and bit 7 corresponds to the most significant byte. For
                                                     // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
                                                     // DATA[63:56] is not valid
      
            .m_axis_tuser(m_axis_tuser),             // TUSER_WIDTH-bit output: TUSER: The user-defined sideband
                                                     // information that can be transmitted alongside the data
                                                     // stream.
      
            .m_axis_tvalid(m_axis_tvalid),           // 1-bit output: TVALID: Indicates that the master is driving a
                                                     // valid transfer. A transfer takes place when both TVALID and
                                                     // TREADY are asserted
      
            .prog_empty_axis(),       // 1-bit output: Programmable Empty- This signal is asserted
                                                     // when the number of words in the FIFO is less than or equal to
                                                     // the programmable empty threshold value. It is de-asserted
                                                     // when the number of words in the FIFO exceeds the programmable
                                                     // empty threshold value.
      
            .prog_full_axis(),         // 1-bit output: Programmable Full: This signal is asserted when
                                                     // the number of words in the FIFO is greater than or equal to
                                                     // the programmable full threshold value. It is de-asserted when
                                                     // the number of words in the FIFO is less than the programmable
                                                     // full threshold value.
      
            .rd_data_count_axis(), // RD_DATA_COUNT_WIDTH-bit output: Read Data Count- This bus
                                                     // indicates the number of words available for reading in the
                                                     // FIFO.
      
            .s_axis_tready(s_axis_tready),           // 1-bit output: TREADY: Indicates that the slave can accept a
                                                     // transfer in the current cycle.
      
            .sbiterr_axis(),             // 1-bit output: Single Bit Error- Indicates that the ECC
                                                     // decoder detected and fixed a single-bit error.
      
            .wr_data_count_axis(), // WR_DATA_COUNT_WIDTH-bit output: Write Data Count: This bus
                                                     // indicates the number of words written into the FIFO.
      
            .injectdbiterr_axis(), // 1-bit input: Double Bit Error Injection- Injects a double bit
                                                     // error if the ECC feature is used.
      
            .injectsbiterr_axis(), // 1-bit input: Single Bit Error Injection- Injects a single bit
                                                     // error if the ECC feature is used.
      
            .m_aclk(clk),                         // 1-bit input: Master Interface Clock: All signals on master
                                                     // interface are sampled on the rising edge of this clock.
      
            .m_axis_tready(m_axis_tready),           // 1-bit input: TREADY: Indicates that the slave can accept a
                                                     // transfer in the current cycle.
      
            .s_aclk(mac_rx_gmii_clk),                 // 1-bit input: Slave Interface Clock: All signals on slave
                                                     // interface are sampled on the rising edge of this clock.
      
            .s_aresetn(!rst),                   // 1-bit input: Active low asynchronous reset.
            .s_axis_tdata(s_axis_tdata),             // TDATA_WIDTH-bit input: TDATA: The primary payload that is
                                                     // used to provide the data that is passing across the
                                                     // interface. The width of the data payload is an integer number
                                                     // of bytes.
      
            .s_axis_tdest(1'b0),             // TDEST_WIDTH-bit input: TDEST: Provides routing information
                                                     // for the data stream.
      
            .s_axis_tid(1'b0),                 // TID_WIDTH-bit input: TID: The data stream identifier that
                                                     // indicates different streams of data.
      
            .s_axis_tkeep(1'b1),             // TDATA_WIDTH/8-bit input: TKEEP: The byte qualifier that
                                                     // indicates whether the content of the associated byte of TDATA
                                                     // is processed as part of the data stream. Associated bytes
                                                     // that have the TKEEP byte qualifier deasserted are null bytes
                                                     // and can be removed from the data stream. For a 64-bit DATA,
                                                     // bit 0 corresponds to the least significant byte on DATA, and
                                                     // bit 7 corresponds to the most significant byte. For example:
                                                     // KEEP[0] = 1b, DATA[7:0] is not a NULL byte KEEP[7] = 0b,
                                                     // DATA[63:56] is a NULL byte
      
            .s_axis_tlast(s_axis_tlast),             // 1-bit input: TLAST: Indicates the boundary of a packet.
            .s_axis_tstrb(1'b1),             // TDATA_WIDTH/8-bit input: TSTRB: The byte qualifier that
                                                     // indicates whether the content of the associated byte of TDATA
                                                     // is processed as a data byte or a position byte. For a 64-bit
                                                     // DATA, bit 0 corresponds to the least significant byte on
                                                     // DATA, and bit 0 corresponds to the least significant byte on
                                                     // DATA, and bit 7 corresponds to the most significant byte. For
                                                     // example: STROBE[0] = 1b, DATA[7:0] is valid STROBE[7] = 0b,
                                                     // DATA[63:56] is not valid
      
            .s_axis_tuser(s_axis_tuser),             // TUSER_WIDTH-bit input: TUSER: The user-defined sideband
                                                     // information that can be transmitted alongside the data
                                                     // stream.
      
            .s_axis_tvalid(s_axis_tvalid)            // 1-bit input: TVALID: Indicates that the master is driving a
                                               // valid transfer. A transfer takes place when both TVALID and
                                               // TREADY are asserted

        );
    end
endgenerate
    
endmodule