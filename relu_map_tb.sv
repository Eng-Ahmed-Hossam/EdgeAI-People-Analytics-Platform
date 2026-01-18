`timescale 1ns/1ps

module tb_relu_map_2ch;

    parameter CH = 2;
    parameter IMG_SIZE = 4;
    parameter DATA_W = 32;

    logic clk, rst, in_valid;
    logic signed [DATA_W-1:0] in_map  [CH][IMG_SIZE][IMG_SIZE];
    logic signed [DATA_W-1:0] out_map [CH][IMG_SIZE][IMG_SIZE];
    logic out_valid, done;

    // -------------------------
    // DUT
    // -------------------------
    relu_map #(
        .CH(CH),
        .IMG_SIZE(IMG_SIZE),
        .DATA_W(DATA_W)
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
    // Test sequence
    // -------------------------
    initial begin
        clk = 0;
        rst = 1;
        in_valid = 0;

        // =========================
        // Input feature maps
        // =========================

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

        // -------------------------
        // Reset
        // -------------------------
        repeat(2) @(posedge clk);
        rst = 0;

        // -------------------------
        // Start ReLU map
        // -------------------------
        @(posedge clk);
        in_valid = 1;

        // Wait until whole map done
        wait(done);

        @(posedge clk);
        in_valid = 0;

        // -------------------------
        // Print results
        // -------------------------
        $display("\n==============================");
        $display(" INPUT FEATURE MAP ");
        $display("==============================");
        for (int c = 0; c < CH; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i < IMG_SIZE; i++) begin
                $write("[ ");
                for (int j = 0; j < IMG_SIZE; j++)
                    $write("%6d ", in_map[c][i][j]);
                $write("]\n");
            end
            $display("");
        end

        $display("\n==============================");
        $display(" RELU OUTPUT FEATURE MAP ");
        $display("==============================");
        for (int c = 0; c < CH; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i < IMG_SIZE; i++) begin
                $write("[ ");
                for (int j = 0; j < IMG_SIZE; j++)
                    $write("%6d ", out_map[c][i][j]);
                $write("]\n");
            end
            $display("");
        end

        $display("=========== DONE  ===========\n");

        #20 $finish;
    end

endmodule
