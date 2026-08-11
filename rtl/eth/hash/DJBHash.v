module DJBHash (
    input         clk     ,
    input         rst     ,
    input  [31:0] data_in ,
    output [ 7:0] data_out 
);

localparam MAGIC = 32'd5381;
localparam PIPE_F = (MAGIC << 32'd5) + MAGIC;

reg [31:0] data_in_ff[3:0];
reg [7:0] pipe [3:0];

always @(posedge clk ) begin
    data_in_ff[0] <= data_in;
end
genvar i;
generate
    for (i = 0; i < 3; i = i + 1) begin : LOOP_HASH
        always @(posedge clk ) begin
            data_in_ff[i + 1] <= data_in_ff[i];
        end
    end
endgenerate

always @(posedge clk ) begin
    pipe[0] <= PIPE_F + data_in[31 -: 8];
    pipe[1] <= (pipe[0] << 3'd5) + pipe[0] + data_in_ff[0][23 -: 8];
    pipe[2] <= (pipe[1] << 3'd5) + pipe[1] + data_in_ff[1][15 -: 8];
    pipe[3] <= (pipe[2] << 3'd5) + pipe[2] + data_in_ff[2][ 7 -: 8];
end
assign data_out = pipe[3][7 -: 8];
    
endmodule