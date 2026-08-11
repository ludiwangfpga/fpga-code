module rgmii_rx #(
    parameter CLOCK_BUFF = "BUFG"  ,
    parameter DEVICE     = "XILINX",
    parameter IDDR_PRIM  = "IODDR" 
) (
    input        clk_200M      ,
    input        phy_rgmii_clk ,
    input        phy_rgmii_ctl ,
    input  [3:0] phy_rgmii_data,

    output [7:0] mac_gmii_data ,
    output       mac_gmii_clk  ,
    output       mac_gmii_dv   ,
    output       mac_gmii_err   

);

generate
    if(DEVICE == "XILINX")begin
        if(CLOCK_BUFF == "BUFG")begin
            BUFG clk_bufg (
                .I(phy_rgmii_clk),
                .O(mac_gmii_clk )
            );
        end
        else begin
            assign mac_gmii_clk = phy_rgmii_clk;
        end
        wire ctrl1;
        wire ctrl2;
        iddr #(
            .DEVICE     (DEVICE   ),
            .IDDR_PRIM  (IDDR_PRIM),
            .DATA_WIDTH (5)
        ) iddr_inst (
            .clk_200M   (clk_200M     ),
            .clk        (phy_rgmii_clk),
            .d          ({phy_rgmii_data, phy_rgmii_ctl}),
        
            .q1         ({mac_gmii_data[3:0], ctrl1}),
            .q2         ({mac_gmii_data[7:4], ctrl2})
        );
        assign mac_gmii_dv = ctrl1;
        assign mac_gmii_err = ctrl1 ^ ctrl2;
    end
endgenerate
    
endmodule