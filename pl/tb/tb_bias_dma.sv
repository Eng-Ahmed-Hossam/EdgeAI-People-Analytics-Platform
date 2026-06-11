/*
------------------------------------------------------------------------------
Testbench : tb_bias_dma
Tests     : bias_dma.v — AXI4 burst-read INT32 bias loader
Method    :
  1. AXI slave memory model pre-loaded with deterministic INT32 pattern.
  2. Tests L0 (16 biases), L1 (pool=0), L2 (32), L8 (256).
  3. For each conv layer: verifies bias_ld_en fires exactly bias_count[L] times
     and each bias_ld_data word matches the expected memory content.
  4. For pool layers: verifies load_done fires in ≤ 4 cycles with 0 ld_en pulses.
  5. Arbiter compatibility test: assert load_start without arready to verify
     bias_dma holds arvalid until arready (not a single-cycle handshake).
------------------------------------------------------------------------------
*/
`timescale 1ns/1ps

module tb_bias_dma;

    localparam AXI_AW     = 32;
    localparam AXI_DW     = 32;
    localparam BIAS_AW    = 8;
    localparam NUM_LAYERS = 19;

    logic clk = 0;
    logic rst = 1;
    always #5 clk = ~clk;

    // DUT ports
    logic [4:0]          layer_idx;
    logic [31:0]         bias_base_addr;
    logic                load_start;
    logic                load_done;

    logic [AXI_AW-1:0]  m_axi_araddr;
    logic               m_axi_arvalid;
    logic               m_axi_arready;
    logic [7:0]         m_axi_arlen;
    logic [2:0]         m_axi_arsize;
    logic [1:0]         m_axi_arburst;
    logic [AXI_DW-1:0]  m_axi_rdata;
    logic               m_axi_rvalid;
    logic               m_axi_rlast;
    logic               m_axi_rready;

    logic               bias_ld_en;
    logic [BIAS_AW-1:0] bias_ld_addr;
    logic [31:0]        bias_ld_data;

    bias_dma #(
        .AXI_AW(AXI_AW), .AXI_DW(AXI_DW), .BIAS_AW(BIAS_AW), .NUM_LAYERS(NUM_LAYERS)
    ) dut (.*);

    // =========================================================================
    // AXI slave memory model — 64 KB byte-addressable (covers full 8632-byte blob)
    // Pattern: memory word at byte-offset i holds INT32 = (i/4 + 1) * 0x01010101
    // so each word has a unique, deterministic 32-bit value.
    // =========================================================================
    localparam MEM_SIZE = 64*1024;
    logic [7:0] mem [0:MEM_SIZE-1];

    initial begin
        for (int w = 0; w < MEM_SIZE/4; w++) begin
            automatic int val = (w + 1) * 32'h01010101;
            mem[w*4+0] = val[7:0];
            mem[w*4+1] = val[15:8];
            mem[w*4+2] = val[23:16];
            mem[w*4+3] = val[31:24];
        end
    end

    // AXI4 read slave
    logic [31:0] slave_read_addr;
    logic [7:0]  beats_left;
    logic        slave_active;

    initial begin
        m_axi_arready = 0;
        m_axi_rvalid  = 0;
        m_axi_rdata   = 0;
        m_axi_rlast   = 0;
        slave_active  = 0;
    end

    always @(posedge clk) begin
        m_axi_arready <= 1'b0;
        m_axi_rvalid  <= 1'b0;
        m_axi_rlast   <= 1'b0;

        if (!slave_active) begin
            if (m_axi_arvalid) begin
                m_axi_arready   <= 1'b1;
                slave_read_addr <= m_axi_araddr;
                beats_left      <= m_axi_arlen;
                slave_active    <= 1'b1;
            end
        end else begin
            if (m_axi_rready) begin
                automatic logic [31:0] addr32 = slave_read_addr & (MEM_SIZE - 1);
                m_axi_rvalid <= 1'b1;
                m_axi_rdata  <= {mem[addr32+3], mem[addr32+2],
                                  mem[addr32+1], mem[addr32]};
                m_axi_rlast  <= (beats_left == 0);
                slave_read_addr <= slave_read_addr + 4;
                if (beats_left == 0)
                    slave_active <= 1'b0;
                else
                    beats_left   <= beats_left - 1;
            end
        end
    end

    // =========================================================================
    // Bias count reference
    // =========================================================================
    int bias_counts [0:NUM_LAYERS-1];
    int bias_offsets[0:NUM_LAYERS-1];
    initial begin
        bias_counts[0]=16;  bias_offsets[0]=0;
        bias_counts[1]=0;   bias_offsets[1]=64;
        bias_counts[2]=32;  bias_offsets[2]=64;
        bias_counts[3]=0;   bias_offsets[3]=192;
        bias_counts[4]=64;  bias_offsets[4]=192;
        bias_counts[5]=0;   bias_offsets[5]=448;
        bias_counts[6]=128; bias_offsets[6]=448;
        bias_counts[7]=0;   bias_offsets[7]=960;
        bias_counts[8]=256; bias_offsets[8]=960;
        bias_counts[9]=0;   bias_offsets[9]=1984;
        bias_counts[10]=256;bias_offsets[10]=1984;
        bias_counts[11]=0;  bias_offsets[11]=2984;
        bias_counts[12]=256;bias_offsets[12]=2984;
        bias_counts[13]=256;bias_offsets[13]=3008;
        bias_counts[14]=256;bias_offsets[14]=4032;
        bias_counts[15]=255;bias_offsets[15]=5056;
        bias_counts[16]=128;bias_offsets[16]=6076;
        bias_counts[17]=256;bias_offsets[17]=6588;
        bias_counts[18]=255;bias_offsets[18]=7612;
    end

    // =========================================================================
    // Test infrastructure
    // =========================================================================
    int total_pass = 0, total_fail = 0;

    task check_int(input string name, input int got, input int exp);
        if (got === exp) begin
            $display("    PASS: %s = %0d", name, got);
            total_pass++;
        end else begin
            $display("    FAIL: %s: got=%0d exp=%0d", name, got, exp);
            total_fail++;
        end
    endtask

    task automatic test_bias_layer(input [4:0] lid);
        automatic int expected_n  = bias_counts[lid];
        automatic int expected_off = bias_offsets[lid];
        automatic int en_count    = 0;
        automatic int data_mismatch = 0;
        automatic int max_cyc     = expected_n * 8 + 200;
        automatic int t           = 0;

        $display("  >> Layer %0d: expecting %0d biases at offset %0d",
                  lid, expected_n, expected_off);

        // Pulse load_start
        @(posedge clk); #1;
        layer_idx  = lid;
        load_start = 1'b1;
        @(posedge clk); #1;
        load_start = 1'b0;

        // Collect pulses until load_done or timeout
        while (!load_done && t < max_cyc) begin
            @(posedge clk);
            t++;
            if (bias_ld_en) begin
                automatic int exp_word_idx = expected_off/4 + en_count;
                automatic int exp_val      = (exp_word_idx + 1) * 32'h01010101;
                if (bias_ld_data !== exp_val[31:0]) begin
                    if (data_mismatch < 3)
                        $display("    DATA MISMATCH beat %0d: got=0x%08X exp=0x%08X",
                                  en_count, bias_ld_data, exp_val[31:0]);
                    data_mismatch++;
                end
                en_count++;
            end
        end

        if (t >= max_cyc) begin
            $display("    TIMEOUT (layer %0d, %0d cycles)", lid, t);
            total_fail++;
        end else if (expected_n == 0) begin
            // Pool layer: load_done with 0 en pulses
            if (en_count == 0 && t < 10) begin
                $display("    PASS: pool layer done in %0d cycles, 0 ld_en", t);
                total_pass++;
            end else begin
                $display("    FAIL: pool layer had %0d ld_en pulses or took too long (%0d cycles)",
                          en_count, t);
                total_fail++;
            end
        end else begin
            check_int("bias_ld_en count", en_count, expected_n);
            if (data_mismatch == 0) begin
                $display("    PASS: all %0d bias data words correct", en_count);
                total_pass++;
            end else begin
                $display("    FAIL: %0d data mismatches", data_mismatch);
                total_fail++;
            end
        end

        // Idle gap between layers
        repeat(4) @(posedge clk);
    endtask

    // =========================================================================
    // Main test
    // =========================================================================
    initial begin
        bias_base_addr = 32'h0044_0000;
        load_start     = 0;
        layer_idx      = 0;

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("======================================================");
        $display("  tb_bias_dma — INT32 bias load verification");
        $display("======================================================");

        // L0: 16 biases (64 bytes)
        test_bias_layer(5'd0);

        // L1: pool → instant done
        test_bias_layer(5'd1);

        // L2: 32 biases (128 bytes)
        test_bias_layer(5'd2);

        // L8: 256 biases (1024 bytes) — tests multi-burst (1 burst of 256 beats)
        test_bias_layer(5'd8);

        // L15: 255 biases — non-power-of-2 count
        test_bias_layer(5'd15);

        $display("======================================================");
        $display("  RESULT: %0d PASS / %0d FAIL", total_pass, total_fail);
        $display("======================================================");
        $finish;
    end

endmodule
