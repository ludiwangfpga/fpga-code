module arbiter_base # (
    parameter REQ_NUM = 4
) (
    input  [REQ_NUM - 1:0] req  ,
    input  [REQ_NUM - 1:0] base ,
    output [REQ_NUM - 1:0] grant 
);

wire [2*REQ_NUM - 1:0] d_req;
wire [2*REQ_NUM - 1:0] d_grant;
assign d_req = {req, req};
assign d_grant = d_req & (~(d_req - base));
assign grant = d_grant[REQ_NUM - 1:0] | d_grant[2*REQ_NUM - 1:REQ_NUM];
    
endmodule