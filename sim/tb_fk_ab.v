`timescale 1ns / 1ps
// tb_fk_ab.v — R1.2 官方复现管线②: fk_engine 直连精度测量 testbench (A/B 同管线)。
// 读 sim/fk_ab_vectors.mem (1004 例 × 6 关节角, Q16.16), 逐例馈入 fk_engine,
// 等 done 后把位姿 6 元组 (x,y,z,rx,ry,rz 原始 Q16.16) 写 fk_ab_out.txt,
// 由 tools/fk_ab_check.py 对双精度参考统计误差。不走 UDP 命令路径。
module tb_fk_ab;

    localparam N_CASES = 1004;

    reg clk = 0;
    reg rst_n = 0;
    reg start = 0;
    reg signed [31:0] q0, q1, q2, q3, q4, q5;
    wire signed [31:0] pose_x, pose_y, pose_z;
    wire signed [31:0] pose_rx, pose_ry, pose_rz;
    wire done;

    fk_engine dut (
        .clk(clk), .rst_n(rst_n), .start(start),
        .q0(q0), .q1(q1), .q2(q2), .q3(q3), .q4(q4), .q5(q5),
        .pose_x(pose_x), .pose_y(pose_y), .pose_z(pose_z),
        .pose_rx(pose_rx), .pose_ry(pose_ry), .pose_rz(pose_rz),
        .done(done)
    );

    always #4 clk = ~clk;   // 125 MHz

    reg [31:0] vec [0:N_CASES*6-1];
    integer i, fd;

    initial begin
        $readmemh("sim/fk_ab_vectors.mem", vec);
        fd = $fopen("fk_ab_out.txt", "w");
        rst_n = 0;
        repeat (4) @(posedge clk);
        rst_n = 1;
        repeat (2) @(posedge clk);
        for (i = 0; i < N_CASES; i = i + 1) begin
            q0 = $signed(vec[i*6+0]); q1 = $signed(vec[i*6+1]); q2 = $signed(vec[i*6+2]);
            q3 = $signed(vec[i*6+3]); q4 = $signed(vec[i*6+4]); q5 = $signed(vec[i*6+5]);
            @(posedge clk); start <= 1;
            @(posedge clk); start <= 0;
            @(posedge clk);
            while (!done) @(posedge clk);
            $fdisplay(fd, "%0d %0d %0d %0d %0d %0d",
                      pose_x, pose_y, pose_z, pose_rx, pose_ry, pose_rz);
        end
        $fclose(fd);
        $display("FK_AB_DUMP_OK %0d", N_CASES);
        $finish;
    end

endmodule
