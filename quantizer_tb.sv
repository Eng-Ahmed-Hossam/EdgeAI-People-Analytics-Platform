`timescale 1ns/1ps

module tb_quantizer;

    logic clk;
    logic signed [31:0] in;
    logic in_valid;

    logic signed [7:0] out;
    logic out_valid;

    // Instantiate DUT
    quantize_int32_to_int8 #(.SHIFT(8)) dut (
        .in(in),
        .in_valid(in_valid),
        .out(out),
        .out_valid(out_valid)
    );

    // clock (not required, but useful for timing)
    initial clk = 0;
    always #5 clk = ~clk;

    // Test procedure
    initial begin
        $display("\n====== QUANTIZER TEST START ======\n");

        in_valid = 0;
        in = 0;
        #10;

        // ---------------------------------------------------
        // CASE 1: zero
        // ---------------------------------------------------
        in_valid = 1;
        in = 32'sd0;
        #10;
        $display("IN = %0d  -> OUT = %0d (expect 0)", in, out);

        // ---------------------------------------------------
        // CASE 2: small positive (no saturation)
        // ---------------------------------------------------
        in = 32'sd2133;  // 1024 / 256 = 4
        #10;
        $display("IN = %0d  -> OUT = %0d ", in, out);

        // ---------------------------------------------------
        // CASE 3: small negative (no saturation)
        // ---------------------------------------------------
        in = -32'sd7641;    
        #10;
        $display("IN = %0d -> OUT = %0d ", in, out);

        // ---------------------------------------------------
        // CASE 4: large positive (saturation)
        // ---------------------------------------------------
        in = 32'sd100000;  // 100000 / 256 = 390 → >127
        #10;
        $display("IN = %0d -> OUT = %0d (expect 127, clipped)", in, out);

        // ---------------------------------------------------
        // CASE 5: large negative (saturation)
        // ---------------------------------------------------
        in = -32'sd90000;  // -90000 / 256 = -351 → < -128
        #10;
        $display("IN = %0d -> OUT = %0d (expect -128, clipped)", in, out);

        // ---------------------------------------------------
        // CASE 6: in_valid = 0
        // ---------------------------------------------------
        in_valid = 0;
        in = 32'sd5000;
        #10;
        $display("IN_VALID = 0 -> OUT = %0d , OUT_VALID = %b (expect 0, 0)", out, out_valid);

        $display("\n====== QUANTIZER TEST END ======\n");
        $stop;
    end

endmodule
