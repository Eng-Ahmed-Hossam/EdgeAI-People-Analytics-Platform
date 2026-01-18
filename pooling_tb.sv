`timescale 1ns/1ps

module tb_max_pooling;

    // Parameters
    parameter IMG_SIZE = 4;
    parameter CH_IN    = 3;
    parameter CH_OUT   = 3;
    parameter K        = 2;
    parameter STRIDE   = 2;
    localparam OUT_SIZE = (IMG_SIZE-K)/STRIDE + 1;  // = 2

    // DUT I/O
    logic clk, rst, in_valid;
    logic signed [7:0] img [CH_IN][IMG_SIZE][IMG_SIZE];
    logic signed [7:0] pooled_output [CH_OUT][OUT_SIZE][OUT_SIZE];
    logic out_valid, done;

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk;

    // Instantiate DUT
    max_pooling dut (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .x(img),
        .pooled_output(pooled_output),
        .out_valid(out_valid),
        .done(done)
    );

    // ---------------------------
    // Initialize test
    // ---------------------------
    initial begin
        rst = 1;
        in_valid = 0;

        // Channel 0
        img[0] = '{ '{1,2,3,4},
                    '{5,6,7,8},
                    '{9,10,11,12},
                    '{13,14,15,16} };

        // Channel 1
        img[1] = '{ '{16,15,14,13},
                    '{12,11,10,9},
                    '{8,7,6,5},
                    '{4,3,2,1} };

        // Channel 2
        img[2] = '{ '{1,3,5,7},
                    '{2,4,6,8},
                    '{9,11,13,15},
                    '{10,12,14,16} };

        // Hold reset for 3 cycles
        repeat(3) @(posedge clk);
        rst = 0;

        // Enable pooling
        @(posedge clk);
        in_valid = 1;

        // Wait until pooling is done
        wait(done);
        @(posedge clk);
        in_valid = 0; // disable further processing

        // Print input images
        $display("\n========== INPUT IMAGES ==========\n");
        for (int c = 0; c < CH_IN; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i < IMG_SIZE; i++) begin
                $write("[ ");
                for (int j = 0; j < IMG_SIZE; j++)
                    $write("%0d ", img[c][i][j]);
                $write("]\n");
            end
            $display("");
        end

        // Print pooled output
        $display("\n========== POOLED OUTPUT ==========\n");
        for (int c = 0; c < CH_OUT; c++) begin
            $display("Output Channel %0d:", c);
            for (int i = 0; i < OUT_SIZE; i++) begin
                $write("[ ");
                for (int j = 0; j < OUT_SIZE; j++)
                    $write("%0d ", pooled_output[c][i][j]);
                $write("]\n");
            end
            $display("");
        end

        $display("\n======== Simulation Finished ========\n");
        $finish;
    end

endmodule
