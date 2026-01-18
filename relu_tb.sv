`timescale 1ns/1ps

module tb_relu;

    // Parameters
    parameter DATA_W = 32;

    // Signals
    logic signed [DATA_W-1:0] in;
    logic in_valid;
    logic signed [DATA_W-1:0] out;
    logic out_valid;

    // Instantiate DUT
    relu #(
        .DATA_W(DATA_W)
    ) dut (
        .in(in),
        .in_valid(in_valid),
        .out(out),
        .out_valid(out_valid)
    );

    // Clock (not needed since combinational, but for timing control)
    logic clk = 0;
    always #5 clk = ~clk;

    // Test stimulus
    initial begin
        $display("===== Starting ReLU Testbench =====");

        // Test 1: positive number
        in = 32'd10;
        in_valid = 1;
        #1; // small delay for combinational logic
        $display("Input: %0d, in_valid: %b => Output: %0d, out_valid: %b", in, in_valid, out, out_valid);

        // Test 2: zero
        in = 32'd0;
        #1;
        $display("Input: %0d, in_valid: %b => Output: %0d, out_valid: %b", in, in_valid, out, out_valid);

        // Test 3: negative number
        in = -32'd15;
        #1;
        $display("Input: %0d, in_valid: %b => Output: %0d, out_valid: %b", in, in_valid, out, out_valid);

        // Test 4: in_valid low
        in_valid = 0;
        in = 32'd25;
        #1;
        $display("Input: %0d, in_valid: %b => Output: %0d, out_valid: %b", in, in_valid, out, out_valid);

        $display("===== ReLU Testbench Finished =====");
       // $finish;
    end

endmodule
