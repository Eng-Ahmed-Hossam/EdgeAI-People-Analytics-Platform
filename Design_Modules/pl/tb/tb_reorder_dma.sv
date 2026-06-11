/*
------------------------------------------------------------------------------
Testbench : tb_reorder_dma
Tests     : reorder_dma.v — tile-major → channel-major scatter
Method    :
  1. Uses the exact L0 golden parameters (C=16, H=128, W=128).
  2. Generates a known tile-major stream (value = channel*256 + y*128 + x,
     truncated to 8 bits, matching the reorder formula).
  3. Drives stream_valid and streams all 262144 bytes into reorder_dma.
  4. Waits for stream_last, then pulses reorder_start.
  5. Captures all ifmap_wr_en writes into a verification array.
  6. Computes expected scatter addresses using the Python-validated formula:
       ch = tile*PE_ROWS + row
       addr = ch*H*W + y*W + x
  7. Checks: correct byte at correct address for all 262144 writes.

  Passes only when 0 mismatches — same golden guarantee as tb_chained.sv.
------------------------------------------------------------------------------
*/
`timescale 1ns/1ps

module tb_reorder_dma;

    localparam DATA_WIDTH = 8;
    localparam PE_ROWS    = 16;
    localparam C          = 16;     // L0 output channels
    localparam H          = 128;
    localparam W          = 128;
    localparam TOTAL      = C*H*W;  // 262144
    localparam NTILE      = (C + PE_ROWS - 1) / PE_ROWS;  // 1 for C=16
    localparam STAGE_AW   = 18;
    localparam IFMAP_AW   = 18;

    logic clk = 0;
    logic rst = 1;
    always #5 clk = ~clk;

    // DUT signals
    logic [8:0]  out_channels;
    logic [7:0]  out_dim;
    logic        pool_mode;
    logic        last_layer;
    logic signed [7:0] stream_data;
    logic        stream_valid;
    logic        stream_last;
    logic        reorder_start;
    logic        reorder_done;
    logic        ifmap_wr_en;
    logic [IFMAP_AW-1:0] ifmap_wr_addr;
    logic signed [7:0]   ifmap_wr_data;

    reorder_dma #(
        .DATA_WIDTH(DATA_WIDTH), .PE_ROWS(PE_ROWS),
        .MAX_OUT(262144), .STAGE_AW(STAGE_AW), .IFMAP_AW(IFMAP_AW),
        .MAX_DIM(128), .MAX_CH(256)
    ) dut (
        .clk(clk), .rst(rst),
        .out_channels(out_channels), .out_dim(out_dim),
        .pool_mode(pool_mode), .last_layer(last_layer),
        .stream_data(stream_data), .stream_valid(stream_valid),
        .stream_last(stream_last),
        .reorder_start(reorder_start), .reorder_done(reorder_done),
        .ifmap_wr_en(ifmap_wr_en), .ifmap_wr_addr(ifmap_wr_addr),
        .ifmap_wr_data(ifmap_wr_data)
    );

    // =========================================================================
    // Build expected output: channel-major indexed array
    // =========================================================================
    logic [7:0] expected_chyx [0:TOTAL-1];  // indexed [ch*H*W + y*W + x]
    logic [7:0] captured_chyx [0:TOTAL-1];
    logic       addr_written  [0:TOTAL-1];

    // Expected: stream is tile-major, value = (ch + y + x) & 0xFF for variety
    // Tile-major stream order: tile → y → x → row
    int stream_vals [0:TOTAL-1];

    initial begin
        integer idx;
        idx = 0;
        for (int tile = 0; tile < NTILE; tile++) begin
            for (int y = 0; y < H; y++) begin
                for (int x = 0; x < W; x++) begin
                    for (int row = 0; row < PE_ROWS; row++) begin
                        automatic int ch = tile*PE_ROWS + row;
                        if (ch < C) begin
                            automatic int val = (ch*17 + y*7 + x*3) & 8'hFF;
                            stream_vals[idx] = val;
                            expected_chyx[ch*H*W + y*W + x] = val[7:0];
                            idx++;
                        end
                    end
                end
            end
        end
    end

    // =========================================================================
    // Test
    // =========================================================================
    int mismatches = 0;
    int writes_captured = 0;

    initial begin
        out_channels  = C;
        out_dim       = H;
        pool_mode     = 0;
        last_layer    = 0;
        stream_valid  = 0;
        stream_last   = 0;
        stream_data   = 0;
        reorder_start = 0;
        for (int i=0; i<TOTAL; i++) begin
            captured_chyx[i] = 8'hXX;
            addr_written[i]  = 0;
        end

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("======================================================");
        $display("  tb_reorder_dma — L0 (C=16,H=128,W=128) reorder test");
        $display("======================================================");

        // Phase 1: stream TOTAL bytes into reorder_dma
        $display("  Phase 1: streaming %0d bytes...", TOTAL);
        for (int i = 0; i < TOTAL; i++) begin
            @(posedge clk); #1;
            stream_valid = 1'b1;
            stream_data  = $signed(stream_vals[i][7:0]);
            stream_last  = (i == TOTAL-1) ? 1'b1 : 1'b0;
        end
        @(posedge clk); #1;
        stream_valid = 1'b0;
        stream_last  = 1'b0;
        @(posedge clk);

        // Phase 2: trigger scatter
        $display("  Phase 2: triggering scatter...");
        @(posedge clk); #1;
        reorder_start = 1'b1;
        @(posedge clk); #1;
        reorder_start = 1'b0;

        // Capture scatter writes
        fork
            begin
                automatic int timeout = TOTAL * 4 + 100;
                automatic int t = 0;
                while (!reorder_done && t < timeout) begin
                    @(posedge clk);
                    t++;
                    if (ifmap_wr_en) begin
                        automatic int addr = ifmap_wr_addr;
                        if (addr < TOTAL) begin
                            captured_chyx[addr] = ifmap_wr_data;
                            addr_written[addr]  = 1;
                            writes_captured++;
                        end
                    end
                end
                if (t >= timeout)
                    $display("  TIMEOUT waiting for reorder_done!");
            end
        join

        $display("  Scatter complete. Writes captured: %0d / %0d",
                 writes_captured, TOTAL);

        // Verify
        for (int i = 0; i < TOTAL; i++) begin
            if (!addr_written[i]) begin
                mismatches++;
            end else if (captured_chyx[i] !== expected_chyx[i]) begin
                if (mismatches < 10)
                    $display("  MISMATCH addr=%0d: got=%02x exp=%02x",
                              i, captured_chyx[i], expected_chyx[i]);
                mismatches++;
            end
        end

        $display("======================================================");
        if (mismatches == 0)
            $display("  RESULT: PASS — 0 mismatches, %0d bytes correct", TOTAL);
        else
            $display("  RESULT: FAIL — %0d mismatches out of %0d", mismatches, TOTAL);
        $display("======================================================");
        $finish;
    end

endmodule
