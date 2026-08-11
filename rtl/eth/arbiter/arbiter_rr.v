module arbiter_rr #(
    parameter REQ_NUM = 4
) (
    input                  clk  ,
    input                  rst  ,
    input  [REQ_NUM - 1:0] req  ,
    output [REQ_NUM - 1:0] grant 
);

reg [REQ_NUM - 1:0] temp;

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        temp <= {{(REQ_NUM - 1){1'b0}}, 1'b1};
    end
    else begin
        if(|req)begin
            temp <= {grant[REQ_NUM - 2:0], grant[REQ_NUM - 1]};
        end
    end
end

arbiter_base # (
    .REQ_NUM (REQ_NUM)
) arbiter_base_inst (
    .req     (req  ),
    .base    (temp ),
    .grant   (grant) 
);
endmodule