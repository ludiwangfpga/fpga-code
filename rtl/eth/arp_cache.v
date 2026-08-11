module arp_cache #(
    parameter FLUSH_TIME = 32'hFFFF_FFFF,
    parameter USE_FLUSH  = "TRUE"
) (
    input         clk    ,
    input         rst    ,
    input         w_valid,
    input  [31:0] wip    ,
    input  [47:0] wmac   ,
    input         r_valid,
    input  [31:0] rip    ,
    output [47:0] rmac   ,
    output [ 1:0] hit    , //01-->hit 10-->miss 
    output        ready   
);

wire [7:0] hash_w;
wire [7:0] hash_r;

reg [3:0]  w_valid_ff;
reg [4:0]  r_valid_ff;
reg [31:0] wip_ff  [3:0];
reg [47:0] wmac_ff [3:0];
reg [31:0] rip_ff  [4:0];

reg [31:0] ip_mem  [255:0];
reg [47:0] mac_mem [255:0];

wire wen;
wire ren;
wire ren_re;
reg  ren_ff;
reg  ren_re_ff;
wire [7:0] w_addr;
reg [7:0] r_addr;
wire [31:0] w_ip;
wire [47:0] w_mac;

reg [31:0] r_ip ;
reg [47:0] r_mac;

reg  [31:0] r_ip_re ;
reg  [47:0] r_mac_re;

reg  [ 1:0] hit_r  ;
reg  [47:0] hit_mac;

generate
    if(USE_FLUSH == "TRUE")begin        
        wire flush;
        reg [31:0] cnt;
        reg [ 7:0] cnt1;
        
        always @(posedge clk ) begin
            if(rst == 1'b1)begin
                cnt <= 32'd0;
            end
            else begin
                if(cnt == FLUSH_TIME)begin
                    if(cnt1 == 8'hFF)begin
                        cnt <=  32'd0;
                    end
                end
                else begin
                    cnt <= cnt + 1'b1;
                end
            end
        end

        always @(posedge clk ) begin
            if(rst == 1'b1)begin
                cnt1 <= 8'd0;
            end
            else if(cnt == FLUSH_TIME)begin
                cnt1 <= cnt1 + 8'd1;
            end
            else begin
                cnt1 <= 8'd0;
            end
        end
        assign ready  = cnt != FLUSH_TIME;
        assign w_ip   = cnt != FLUSH_TIME ? wip_ff[3]  : 32'd0;
        assign w_mac  = cnt != FLUSH_TIME ? wmac_ff[3] : 48'd0;
        assign w_addr = cnt != FLUSH_TIME ? hash_w : cnt1;
        assign wen =    cnt != FLUSH_TIME ? w_valid_ff[3] : 1'b1;

    end
    else begin
        assign ready = 1'b1;
        assign w_ip  = wip_ff[3] ;
        assign w_mac = wmac_ff[3];
        assign w_addr = hash_w;
        assign wen = w_valid_ff[3];
    end
endgenerate



always @(posedge clk ) begin
    if(rst == 1'b1)begin
        w_valid_ff <= 4'b0;
        r_valid_ff <= 5'b0;
    end
    else begin
        w_valid_ff <= {w_valid_ff[2:0], w_valid};
        r_valid_ff <= {r_valid_ff[3:0], r_valid};
    end
end

always @(posedge clk ) begin
    wip_ff[0]  <= wip ;
    wmac_ff[0] <= wmac;
    rip_ff[0]  <= rip ;
end

genvar i;
generate
    for (i = 0; i < 3; i = i + 1) begin : LOOP_W
        always @(posedge clk ) begin
            wip_ff[i+1]  <= wip_ff[i] ;
            wmac_ff[i+1] <= wmac_ff[i];
        end
    end
endgenerate

genvar j;
generate
    for (j = 0; j < 4; j = j + 1) begin : LOOP_R
        always @(posedge clk ) begin
            rip_ff[j+1]  <= rip_ff[j] ;
        end
    end
endgenerate

assign ren = r_valid_ff[3] && ((!w_valid_ff[3]) || ((w_valid_ff[3] == 1'b1) && (hash_r != hash_w)));
assign ren_re = r_valid_ff[3] && ((w_valid_ff[3] == 1'b1) && (hash_r == hash_w));

always @(posedge clk ) begin
    if(ren_re == 1'b1)begin
        r_ip_re  <= wip_ff[3];
        r_mac_re <= wmac_ff[3];
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        ren_ff <= 1'b0;
        ren_re_ff <= 1'b0;
    end
    else begin
        ren_ff <= ren;
        ren_re_ff <= ren_re;
    end
end

always @(posedge clk ) begin
    if(rst == 1'b1)begin
        hit_r <= 2'b0;
        hit_mac <= 48'd0;
    end
    else begin
        if(ren_ff == 1'b1)begin
            if(r_ip == rip_ff[4])begin
                hit_r <= 2'b01;
                hit_mac <= r_mac;
            end
            else begin
                hit_r <= 2'b10;
                hit_mac <= 48'd0;
            end
        end
        else if(ren_re_ff == 1'b1)begin
            if(r_ip == r_ip_re)begin
                hit_r <= 2'b01;
                hit_mac <= r_mac_re;
            end
            else begin
                hit_r <= 2'b10;
                hit_mac <= 48'd0;
            end
        end
        else begin
            hit_r <= 2'b0;
            hit_mac <= 48'd0;
        end
    end
end

assign hit  = hit_r;
assign rmac = hit_mac;


DJBHash DJBHash_inst_w(
    .clk         (clk    ),
    .rst         (rst    ),
    .data_in     (wip    ),
    .data_out    (hash_w ) 
);

DJBHash DJBHash_inst_r(
    .clk         (clk    ),
    .rst         (rst    ),
    .data_in     (rip    ),
    .data_out    (hash_r ) 
);

always @(posedge clk ) begin
    if(wen == 1'b1)begin
       ip_mem[w_addr]  <= w_ip ;
       mac_mem[w_addr] <= w_mac;
    end
end

always @(posedge clk ) begin
    if(ren == 1'b1)begin
        r_ip  <= ip_mem[hash_r] ;
        r_mac <= mac_mem[hash_r];
    end
end
    
endmodule