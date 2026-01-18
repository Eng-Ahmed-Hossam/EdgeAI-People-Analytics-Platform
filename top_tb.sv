`timescale 1ns/1ps
module top_tb;

    logic clk;
    logic rst;
    logic in_valid;
    logic signed [7:0] image_in [3][416][416];
    logic done;
    logic signed [7:0] final_out[255][24][24];

    // Instantiate the top module
    top_cnn_24layers uut (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .image_in(image_in),
        .done(done),
        .final_out(final_out)
    );

    // ----------------------
    // Clock generation
    // ----------------------
    initial clk = 0;
    always #5 clk = ~clk; // 100 MHz clock

    // ----------------------
    // Stimulus
    // ----------------------
    initial begin
        // Declare all variables before any statements
        int i, j, c;

        // Initialize inputs
        rst = 1;
        in_valid = 0;

        // Zero the image input
        for (c = 0; c < 3; c++)
            for (i = 0; i < 416; i++)
                for (j = 0; j < 416; j++)
                    image_in[c][i][j] = 0;

        // Wait for a few cycles
        #20;
        rst = 0;

        // Start processing
        #10;
        in_valid = 1;

        #10;
        in_valid = 0;

        // Wait long enough for processing (placeholder)
        #5000;

        // Display a small part of final output
        $display("final_out[0][0][0] = %d", final_out[0][0][0]);
        $display("final_out[0][0][1] = %d", final_out[0][0][1]);

        $finish;
    end

endmodule
