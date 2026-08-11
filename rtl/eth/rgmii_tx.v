module rgmii_tx #(
    parameter DEVICE     = "XILINX",
    parameter IDDR_PRIM  = "IODDR" ,
    parameter CLK_90     = "TRUE" 
) (
    input        clk           ,
    input        clk_90        ,

    output       phy_rgmii_clk ,
    output       phy_rgmii_ctl ,
    output [3:0] phy_rgmii_data,

    input  [7:0] mac_gmii_data ,
    output       mac_gmii_clk  ,
    input        mac_gmii_dv    
);

reg        phy_rgmii_ctl_r ; 
reg  [3:0] phy_rgmii_data_r;
wire       rgmii_ctl_r ; 
wire [3:0] rgmii_data_r;
generate
    if(CLK_90 == "TRUE")begin
        assign mac_gmii_clk = clk_90;
    end
    else begin
        assign mac_gmii_clk = clk;
    end
endgenerate

// assign phy_rgmii_clk = mac_gmii_clk;

// always @(posedge phy_rgmii_clk ) begin
//     phy_rgmii_ctl_r  <= rgmii_ctl_r ;
//     phy_rgmii_data_r <= rgmii_data_r;
// end

// assign phy_rgmii_ctl  = phy_rgmii_ctl_r ;
// assign phy_rgmii_data = phy_rgmii_data_r;

assign phy_rgmii_ctl  = rgmii_ctl_r ;
assign phy_rgmii_data = rgmii_data_r;

generate
    if(DEVICE == "XILINX")begin
        oddr #(
            .DEVICE     (DEVICE   ),
            .IDDR_PRIM  (IDDR_PRIM),
            .DATA_WIDTH (6        )
        ) oddr_inst (
            .clk        (mac_gmii_clk),
            .d1         ({mac_gmii_data[3:0], mac_gmii_dv, 1'b1}),
            .d2         ({mac_gmii_data[7:4], mac_gmii_dv, 1'b0}),
        
            .q          ({rgmii_data_r, rgmii_ctl_r, phy_rgmii_clk}  ) 
        );
    end
endgenerate
    
endmodule