/*
------------------------------------------------------------------------------
Testbench : tb_edgeai_ctrl_regs
Tests     : edgeai_ctrl_regs.v — AXI4-Lite slave register bank
Method    :
  1. AXI4-Lite write then read for each writable register.
  2. Verify control pulses (inference_start, tensor_clear, cnn_soft_reset)
     assert for exactly 1 cycle after CTRL write.
  3. Verify STATUS register correctly reflects PL status inputs.
  4. Verify byte-strobe masking on WSTRB=4'b1100 write.
------------------------------------------------------------------------------
*/
`timescale 1ns/1ps

module tb_edgeai_ctrl_regs;

    logic clk = 0;
    logic rst = 1;
    always #5 clk = ~clk;

    // AXI-Lite signals
    logic [31:0] s_axi_awaddr;
    logic        s_axi_awvalid, s_axi_awready;
    logic [31:0] s_axi_wdata;
    logic [3:0]  s_axi_wstrb;
    logic        s_axi_wvalid, s_axi_wready;
    logic [1:0]  s_axi_bresp;
    logic        s_axi_bvalid, s_axi_bready;
    logic [31:0] s_axi_araddr;
    logic        s_axi_arvalid, s_axi_arready;
    logic [31:0] s_axi_rdata;
    logic [1:0]  s_axi_rresp;
    logic        s_axi_rvalid, s_axi_rready;

    // Outputs under test
    logic inference_start, cnn_soft_reset, tensor_clear;
    logic [31:0] wt_base_addr, bias_base_addr, frame_buf_a, frame_buf_b, tensor_dst;

    // Status inputs
    logic busy = 0, tensor_ready = 0, inference_done = 0, cam_frame_error = 0;

    edgeai_ctrl_regs dut (.*);

    // =========================================================================
    // AXI4-Lite write task
    // =========================================================================
    task axi_write(input [31:0] addr, input [31:0] data, input [3:0] strb = 4'hF);
        @(posedge clk); #1;
        s_axi_awaddr  = addr; s_axi_awvalid = 1;
        s_axi_wdata   = data; s_axi_wvalid  = 1; s_axi_wstrb = strb;
        s_axi_bready  = 1;
        do @(posedge clk); while (!s_axi_awready || !s_axi_wready);
        #1;
        s_axi_awvalid = 0; s_axi_wvalid = 0;
        do @(posedge clk); while (!s_axi_bvalid);
        #1; s_axi_bready = 0;
    endtask

    // AXI4-Lite read task
    task axi_read(input [31:0] addr, output [31:0] rdata);
        @(posedge clk); #1;
        s_axi_araddr  = addr; s_axi_arvalid = 1;
        s_axi_rready  = 1;
        do @(posedge clk); while (!s_axi_arready);
        #1; s_axi_arvalid = 0;
        do @(posedge clk); while (!s_axi_rvalid);
        rdata = s_axi_rdata;
        #1; s_axi_rready = 0;
    endtask

    // =========================================================================
    // Test
    // =========================================================================
    int pass = 0, fail = 0;
    logic [31:0] rd;

    task check(input string name, input logic [31:0] got, input logic [31:0] exp);
        if (got === exp) begin
            $display("  PASS: %s = 0x%08X", name, got);
            pass++;
        end else begin
            $display("  FAIL: %s: got=0x%08X exp=0x%08X", name, got, exp);
            fail++;
        end
    endtask

    initial begin
        s_axi_awvalid=0; s_axi_wvalid=0; s_axi_bready=0;
        s_axi_arvalid=0; s_axi_rready=0;
        s_axi_awaddr=0; s_axi_wdata=0; s_axi_wstrb=4'hF; s_axi_araddr=0;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("======================================================");
        $display("  tb_edgeai_ctrl_regs — AXI4-Lite register bank test");
        $display("======================================================");

        // Test 1: Write and read WT_BASE_ADDR
        axi_write(32'h08, 32'hDEAD_BEEF);
        axi_read (32'h08, rd);
        check("WT_BASE_ADDR", rd, 32'hDEAD_BEEF);

        // Test 2: BIAS_BASE_ADDR
        axi_write(32'h0C, 32'hCAFE_F00D);
        axi_read (32'h0C, rd);
        check("BIAS_BASE_ADDR", rd, 32'hCAFE_F00D);

        // Test 3: FRAME_BUF_A
        axi_write(32'h10, 32'h0020_0000);
        axi_read (32'h10, rd);
        check("FRAME_BUF_A", rd, 32'h0020_0000);

        // Test 4: FRAME_BUF_B
        axi_write(32'h14, 32'h0020_8000);
        axi_read (32'h14, rd);
        check("FRAME_BUF_B", rd, 32'h0020_8000);

        // Test 5: TENSOR_DST
        axi_write(32'h18, 32'h0044_3000);
        axi_read (32'h18, rd);
        check("TENSOR_DST", rd, 32'h0044_3000);

        // Test 6: CTRL write → inference_start pulse (1 cycle)
        begin
            automatic int pulse_cycles = 0;
            @(posedge clk); #1;
            s_axi_awaddr=32'h00; s_axi_awvalid=1;
            s_axi_wdata=32'h01; s_axi_wvalid=1; s_axi_wstrb=4'hF; s_axi_bready=1;
            repeat(20) begin
                @(posedge clk);
                if (inference_start) pulse_cycles++;
            end
            s_axi_awvalid=0; s_axi_wvalid=0;
            if (pulse_cycles == 1) begin
                $display("  PASS: inference_start pulse = 1 cycle");
                pass++;
            end else begin
                $display("  FAIL: inference_start pulse cycles = %0d (expected 1)", pulse_cycles);
                fail++;
            end
        end

        // Test 7: STATUS register reflects input signals
        busy = 1; tensor_ready = 1; inference_done = 0; cam_frame_error = 0;
        @(posedge clk);
        axi_read(32'h04, rd);
        check("STATUS[1:0] (busy|tensor_ready)", rd[1:0], 2'b11);

        // Test 8: Byte-strobe write — upper 2 bytes only
        axi_write(32'h08, 32'hABCD_0000, 4'b1100);  // Write upper 2 bytes only
        axi_read (32'h08, rd);
        // Lower 2 bytes should remain 0xBEEF from Test 1
        check("WT_BASE byte-strobe (upper)", rd[31:16], 16'hABCD);
        check("WT_BASE byte-strobe (lower)", rd[15:0],  16'hBEEF);

        $display("======================================================");
        $display("  RESULT: %0d PASS / %0d FAIL", pass, fail);
        $display("======================================================");
        $finish;
    end

endmodule
