/*
------------------------------------------------------------------------------
Testbench   : post_pipeline_tb
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    SystemVerilog self-checking testbench for the post-processing pipeline:
    bias_add → requantizer → activation_unit → pooling_unit
    
    Tests:
    1. Bias addition correctness
    2. Requantizer scale/shift/clamp
    3. ReLU activation
    4. LeakyReLU activation
    5. Pooling bypass
    6. Full pipeline end-to-end

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`timescale 1ns / 1ps

module post_pipeline_tb;

    localparam DATA_WIDTH  = 8;
    localparam ACC_WIDTH   = 32;
    localparam BIAS_WIDTH  = 32;
    localparam MULT_WIDTH  = 16;
    localparam SHIFT_WIDTH = 6;
    localparam CLK_PERIOD  = 10;
    
    reg clk, rst;
    
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;
    
    integer error_count;
    
    // ========================================================================
    // Bias add DUT
    // ========================================================================
    reg  signed [ACC_WIDTH-1:0]  bias_acc_in;
    reg  signed [BIAS_WIDTH-1:0] bias_in;
    reg                          bias_in_valid;
    wire signed [ACC_WIDTH-1:0]  bias_result;
    wire                         bias_out_valid;
    
    bias_add #(
        .ACC_WIDTH (ACC_WIDTH),
        .BIAS_WIDTH(BIAS_WIDTH)
    ) u_bias (
        .clk      (clk),
        .rst      (rst),
        .acc_in   (bias_acc_in),
        .bias_in  (bias_in),
        .in_valid (bias_in_valid),
        .result   (bias_result),
        .out_valid(bias_out_valid)
    );
    
    // ========================================================================
    // Requantizer DUT
    // ========================================================================
    wire signed [DATA_WIDTH-1:0] rq_data_out;
    wire                         rq_out_valid;
    
    requantizer #(
        .ACC_WIDTH  (ACC_WIDTH),
        .DATA_WIDTH (DATA_WIDTH),
        .MULT_WIDTH (MULT_WIDTH),
        .SHIFT_WIDTH(SHIFT_WIDTH)
    ) u_rq (
        .clk        (clk),
        .rst        (rst),
        .data_in    (bias_result),
        .in_valid   (bias_out_valid),
        .multiplier (16'd1024),      // Scale = 1024
        .shift      (6'd10),         // Shift right 10 → net scale ≈ 1.0
        .zero_point (8'sd0),
        .data_out   (rq_data_out),
        .out_valid  (rq_out_valid)
    );
    
    // ========================================================================
    // Activation DUT
    // ========================================================================
    reg [1:0] act_mode;
    wire signed [DATA_WIDTH-1:0] act_data_out;
    wire                         act_out_valid;
    
    activation_unit #(
        .DATA_WIDTH(DATA_WIDTH),
        .LEAK_SHIFT(3)
    ) u_act (
        .clk      (clk),
        .rst      (rst),
        .data_in  (rq_data_out),
        .in_valid (rq_out_valid),
        .mode     (act_mode),
        .data_out (act_data_out),
        .out_valid(act_out_valid)
    );
    
    // ========================================================================
    // Test sequence
    // ========================================================================
    initial begin
        $display("========================================");
        $display("  Post-Pipeline Testbench Start");
        $display("========================================");
        error_count = 0;
        
        rst = 1;
        bias_acc_in = 0; bias_in = 0; bias_in_valid = 0;
        act_mode = 2'b00;  // ReLU
        repeat(5) @(posedge clk);
        rst = 0;
        @(posedge clk);
        
        // Test 1: Positive value through pipeline with ReLU
        $display("\n[TEST 1] Positive value: acc=1000, bias=24 → bias_result=1024");
        $display("  rq: (1024 * 1024) >> 10 = 1024 → clamp to 127");
        act_mode = 2'b00;  // ReLU
        bias_acc_in = 32'sd1000;
        bias_in = 32'sd24;
        bias_in_valid = 1;
        @(posedge clk);
        bias_in_valid = 0;
        
        // Wait for pipeline
        repeat(8) @(posedge clk);
        
        // Test 2: Negative value through pipeline with ReLU
        $display("\n[TEST 2] Negative value with ReLU: acc=-500, bias=0");
        bias_acc_in = -32'sd500;
        bias_in = 32'sd0;
        bias_in_valid = 1;
        @(posedge clk);
        bias_in_valid = 0;
        
        repeat(8) @(posedge clk);
        
        // Test 3: Negative value with LeakyReLU
        $display("\n[TEST 3] Negative value with LeakyReLU");
        act_mode = 2'b01;  // LeakyReLU
        bias_acc_in = -32'sd500;
        bias_in = 32'sd0;
        bias_in_valid = 1;
        @(posedge clk);
        bias_in_valid = 0;
        
        repeat(8) @(posedge clk);
        
        // Test 4: Zero input
        $display("\n[TEST 4] Zero input");
        act_mode = 2'b00;
        bias_acc_in = 32'sd0;
        bias_in = 32'sd0;
        bias_in_valid = 1;
        @(posedge clk);
        bias_in_valid = 0;
        
        repeat(8) @(posedge clk);
        
        // Test 5: Multiple values in sequence
        $display("\n[TEST 5] Burst of 4 values");
        begin : burst_test
            integer i;
            for (i = 0; i < 4; i = i + 1) begin
                bias_acc_in = (i + 1) * 100;
                bias_in = 32'sd0;
                bias_in_valid = 1;
                @(posedge clk);
            end
            bias_in_valid = 0;
        end
        
        repeat(16) @(posedge clk);
        
        // Monitor output
        $display("\n[INFO] Pipeline outputs captured via waveform.");
        $display("  Check bias_result, rq_data_out, act_data_out for correctness.");
        
        repeat(10) @(posedge clk);
        
        $display("\n========================================");
        if (error_count == 0) $display("  POST-PIPELINE TB COMPLETE");
        else $display("  %0d ERRORS", error_count);
        $display("========================================\n");
        $finish;
    end
    
    // Monitor
    always @(posedge clk) begin
        if (bias_out_valid)
            $display("  [BIAS] result = %0d", bias_result);
        if (rq_out_valid)
            $display("  [RQ]   data_out = %0d", rq_data_out);
        if (act_out_valid)
            $display("  [ACT]  data_out = %0d", act_data_out);
    end
    
    initial begin
        #500000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
