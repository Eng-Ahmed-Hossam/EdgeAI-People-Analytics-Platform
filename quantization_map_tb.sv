`timescale 1ns/1ps

module tb_quant_map;

    parameter CH = 2;
    parameter IMG_SIZE = 4;
    parameter SHIFT = 8;   // divide by 256

    logic clk, rst, in_valid;
    logic signed [31:0] in_map  [CH][IMG_SIZE][IMG_SIZE];
    logic signed [7:0]  out_map [CH][IMG_SIZE][IMG_SIZE];
    logic out_valid, done;

    // -------------------------
    // DUT
    // -------------------------
    quant_map #(
        .CH(CH),
        .IMG_SIZE(IMG_SIZE),
        .SHIFT(SHIFT)
    ) dut (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_map(in_map),
        .out_map(out_map),
        .out_valid(out_valid),
        .done(done)
    );

    // -------------------------
    // Clock
    // -------------------------
    always #5 clk = ~clk;

    // -------------------------
    // Test
    // -------------------------
    initial begin
        clk = 0;
        rst = 1;
        in_valid = 0;

        // -------- Channel 0 --------
        in_map[0][0] = '{ 2463,  3906,  4090,  3180 };
        in_map[0][1] = '{ 2229,  3686,  4471,  4141 };
        in_map[0][2] = '{ 4926,  7082,  6824,  4957 };
        in_map[0][3] = '{ -4469, -5697, -6080, -4501 };

        // -------- Channel 1 --------
        in_map[1][0] = '{ 2296,  2847,  1014,  -280 };
        in_map[1][1] = '{ -1401, -1224, -1787, -2271 };
        in_map[1][2] = '{ -2736, -3809, -4527, -4306 };
        in_map[1][3] = '{ 743,  -138,  -1023, -2223 };

        repeat(2) @(posedge clk);
        rst = 0;

        @(posedge clk);
        in_valid = 1;

        wait(done);

        @(posedge clk);
        in_valid = 0;

        // -------------------------
        // Print results
        // -------------------------
        $display("\n========= QUANT INPUT =========");
        for (int c = 0; c < CH; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i < IMG_SIZE; i++) begin
                for (int j = 0; j < IMG_SIZE; j++)
                    $write("%7d ", in_map[c][i][j]);
                $write("\n");
            end
            $display("");
        end

        $display("\n========= QUANT OUTPUT =========");
        for (int c = 0; c < CH; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i < IMG_SIZE; i++) begin
                for (int j = 0; j < IMG_SIZE; j++)
                    $write("%5d ", out_map[c][i][j]);
                $write("\n");
            end
            $display("");
        end

        $display("======== DONE ✅ ========");
        #20 $finish;
    end

endmodule
