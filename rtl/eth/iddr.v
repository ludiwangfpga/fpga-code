module iddr #(
    parameter DEVICE     = "XILINX",
    parameter IDDR_PRIM  = "IODDR" ,
    parameter DATA_WIDTH = 8
) (
    input                     clk_200M,
    input                     clk,
    input  [DATA_WIDTH - 1:0] d  ,

    output [DATA_WIDTH - 1:0] q1 ,
    output [DATA_WIDTH - 1:0] q2 
);

genvar i;
generate
    if(DEVICE == "XILINX")begin
        wire [DATA_WIDTH - 1:0] delay_data;
        wire                    rgmii_rxc_buf;
        localparam IDELAY_VALUE = 0;
        BUFIO BUFIO_inst (
          .I            (clk),      // 1-bit input: Clock input
          .O            (rgmii_rxc_buf) // 1-bit output: Clock output
        );  
        (* IODELAY_GROUP = "rgmii_rx_delay" *) 
        IDELAYCTRL  IDELAYCTRL_inst (
            .RDY(),                      // 1-bit output: Ready output
            .REFCLK(clk_200M),         // 1-bit input: Reference clock input
            .RST(1'b0)                   // 1-bit input: Active high reset input
        );
        for (i = 0; i < DATA_WIDTH ; i = i + 1) begin
            if(IDDR_PRIM == "IODDR")begin     
                //rgmii_rx_ctrl输入延时与双沿采�?
                (* IODELAY_GROUP = "rgmii_rx_delay" *) 
                IDELAYE2 #(
                  .IDELAY_TYPE     ("FIXED"),           // FIXED, VARIABLE, VAR_LOAD, VAR_LOAD_PIPE
                  .IDELAY_VALUE    (IDELAY_VALUE),      // Input delay tap setting (0-31)
                  .REFCLK_FREQUENCY(200.0)              // IDELAYCTRL clock input frequency in MHz 
                )
                IDELAYE2_inst (
                  .CNTVALUEOUT     (),                  // 5-bit output: Counter value output
                  .DATAOUT         (delay_data[i]),// 1-bit output: Delayed data output
                  .C               (1'b0),              // 1-bit input: Clock input
                  .CE              (1'b0),              // 1-bit input: enable increment/decrement
                  .CINVCTRL        (1'b0),              // 1-bit input: Dynamic clock inversion input
                  .CNTVALUEIN      (5'b0),              // 5-bit input: Counter value input
                  .DATAIN          (1'b0),              // 1-bit input: Internal delay data input
                  .IDATAIN         (d[i]),      // 1-bit input: Data input from the I/O
                  .INC             (1'b0),              // 1-bit input: Increment / Decrement tap delay
                  .LD              (1'b0),              // 1-bit input: Load IDELAY_VALUE input
                  .LDPIPEEN        (1'b0),              // 1-bit input: Enable PIPELINE register
                  .REGRST          (1'b0)               // 1-bit input: Active-high reset tap-delay input
                );
                IDDR #(
                .DDR_CLK_EDGE("SAME_EDGE_PIPELINED"),
                .INIT_Q1(1'b0), // Initial value of Q1: 1'b0 or 1'b1
                .INIT_Q2(1'b0), // Initial value of Q2: 1'b0 or 1'b1
                .SRTYPE("SYNC")
                )
                iddr_inst (
                    .Q1(q1[i]),
                    .Q2(q2[i]),
                    .C(rgmii_rxc_buf),
                    .CE(1'b1),
                    .D(delay_data[i]),
                    .R(1'b0),
                    .S(1'b0)
                );           
            end
            else if(IDDR_PRIM == "IODDRE1")begin
                
            end
        end      
    end
    else begin

    end
endgenerate
    
endmodule