module tpram #(
    parameter DP = 1024,
    parameter WD = 8
) (
    input                             clk  ,
    input        [WD - 1 : 0]         wdata,
    input                             wen  ,
    input        [$clog2(DP) - 1 : 0] waddr,
    input                             ren  ,
    input        [$clog2(DP) - 1 : 0] raddr,
    output reg   [WD - 1 : 0]         rdata

);

reg [WD - 1:0] mem [DP - 1 : 0];
always @(posedge clk ) begin
    if(wen)begin
        mem[waddr] <= wdata;
    end
end
always @(posedge clk ) begin
    if(ren)begin
        rdata <= mem[raddr];
    end
end
    
endmodule