/*
------------------------------------------------------------------------------
Testbench   : pe_tb
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    SystemVerilog self-checking testbench for PE module.
    
    Tests:
    1. Basic MAC: positive × positive 
    2. Signed multiply: negative × positive, negative × negative
    3. Accumulation over multiple cycles
    4. Enable gating
    5. Accumulator clear
    6. Boundary values (INT8 limits)

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps

module pe_tb;

    localparam DATA_WIDTH = 8;
    localparam ACC_WIDTH  = 32;
    localparam CLK_PERIOD = 10;
    
    reg clk, rst, en, acc_clear;
    reg signed [DATA_WIDTH-1:0] ifmap_data, weight_data;
    wire signed [ACC_WIDTH-1:0] acc_out;
    
    pe #(
        .DATA_WIDTH(DATA_WIDTH),
        .ACC_WIDTH (ACC_WIDTH)
    ) dut (
        .clk        (clk),
        .rst        (rst),
        .en         (en),
        .acc_clear  (acc_clear),
        .ifmap_data (ifmap_data),
        .weight_data(weight_data),
        .acc_out    (acc_out)
    );
    
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;
    
    integer error_count;
    
    task check_acc;
        input signed [ACC_WIDTH-1:0] expected;
        input [255:0] test_name;
    begin
        @(posedge clk); #1;
        if (acc_out !== expected) begin
            $display("FAIL [%0s]: Expected %0d, Got %0d", test_name, expected, acc_out);
            error_count = error_count + 1;
        end else begin
            $display("PASS [%0s]: acc_out = %0d", test_name, acc_out);
        end
    end
    endtask
    
    initial begin
        $display("========================================");
        $display("  PE Testbench Start");
        $display("========================================");
        error_count = 0;
        
        rst = 1; en = 0; acc_clear = 0;
        ifmap_data = 0; weight_data = 0;
        repeat(5) @(posedge clk);
        rst = 0;
        @(posedge clk);
        
        // Test 1: Basic MAC  3 × 4 = 12
        $display("\n[TEST 1] Basic MAC: 3 * 4 = 12");
        acc_clear = 1; @(posedge clk); acc_clear = 0;
        en = 1; ifmap_data = 8'sd3; weight_data = 8'sd4;
        check_acc(32'sd12, "3*4");
        
        // Test 2: Accumulate: +12 + 5*6 = 12 + 30 = 42
        $display("\n[TEST 2] Accumulate: 12 + 5*6 = 42");
        ifmap_data = 8'sd5; weight_data = 8'sd6;
        check_acc(32'sd42, "acc+5*6");
        
        // Test 3: Negative multiply: 42 + (-3)*7 = 42 - 21 = 21
        $display("\n[TEST 3] Negative: 42 + (-3)*7 = 21");
        ifmap_data = -8'sd3; weight_data = 8'sd7;
        check_acc(32'sd21, "acc+(-3)*7");
        
        // Test 4: Negative × Negative: 21 + (-2)*(-5) = 21 + 10 = 31
        $display("\n[TEST 4] Neg*Neg: 21 + (-2)*(-5) = 31");
        ifmap_data = -8'sd2; weight_data = -8'sd5;
        check_acc(32'sd31, "acc+(-2)*(-5)");
        
        // Test 5: Enable gating — hold value
        $display("\n[TEST 5] Enable gating (hold)");
        en = 0; ifmap_data = 8'sd100; weight_data = 8'sd100;
        check_acc(32'sd31, "hold");
        
        // Test 6: Accumulator clear
        $display("\n[TEST 6] Accumulator clear");
        acc_clear = 1; @(posedge clk); acc_clear = 0;
        @(posedge clk); #1;
        if (acc_out !== 0) begin
            $display("FAIL [clear]: Expected 0, Got %0d", acc_out);
            error_count = error_count + 1;
        end else begin
            $display("PASS [clear]: acc_out = 0");
        end
        
        // Test 7: Boundary values (127 * 127 = 16129)
        $display("\n[TEST 7] Boundary: 127 * 127 = 16129");
        en = 1; ifmap_data = 8'sd127; weight_data = 8'sd127;
        check_acc(32'sd16129, "127*127");
        
        // Test 8: Boundary: -128 * 127 = -16256
        $display("\n[TEST 8] Boundary: clear then -128 * 127 = -16256");
        acc_clear = 1; @(posedge clk); acc_clear = 0;
        ifmap_data = -8'sd128; weight_data = 8'sd127;
        check_acc(-32'sd16256, "-128*127");
        
        en = 0;
        repeat(5) @(posedge clk);
        
        $display("\n========================================");
        if (error_count == 0) $display("  ALL TESTS PASSED");
        else $display("  %0d TESTS FAILED", error_count);
        $display("========================================\n");
        $finish;
    end
    
    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
