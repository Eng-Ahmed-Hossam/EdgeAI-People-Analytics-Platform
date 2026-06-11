/*
================================================================================
  tb_layer0.sv  —  Layer 0 Verification Testbench  (Phase 9)

  Direct verification of cnn_core.v for Layer 0:
    - Conv 3→16, 3×3, stride=1, pad=1, LeakyReLU
    - Input:   layer0_input.hex       (3×128×128 = 49,152 INT8 bytes)
    - Weights: layer0_weights.hex     (tiled, 432 bytes: 1 tile×27 positions×16 rows)
    - Biases:  layer0_bias.hex        (16×INT32)
    - Golden:  layer0_expected_rtl_order.hex
               ([oy][ox][oc] order = 262,144 bytes matching RTL stream)

  Method:
    1. $readmemh loads into internal memories via hierarchical paths
    2. compute_start pulsed for 1 cycle
    3. out_valid/out_data stream captured in rtl_out[]
    4. After compute_done + pipeline drain, compare vs golden
    5. PASS if total_mismatches == 0

  Expected runtime: ~8.8M cycles (~88 ms at 100 MHz) for L0.
  Timeout set to 15M cycles.
================================================================================
*/

`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_layer0;

    // ========================================================================
    // Clock and reset
    // ========================================================================
    localparam CLK_PERIOD = 10;  // 100 MHz
    reg clk, rst;

    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ========================================================================
    // DUT signals
    // ========================================================================
    // Layer 0 config
    localparam IN_CH   = 9'd3;
    localparam OUT_CH  = 9'd16;
    localparam K_SIZE  = 2'd3;
    localparam IMG_DIM = 8'd128;

    // Output capture — use plain integer to avoid 8-bit * 8-bit = 0 overflow
    localparam integer OUT_TOTAL = 16 * 128 * 128;   // 262,144

    reg  [8:0]  in_channels;
    reg  [8:0]  out_channels;
    reg  [1:0]  kernel_size;
    reg         stride;
    reg         pad_en;
    reg  [1:0]  act_mode;
    reg         pool_en;
    reg  [7:0]  img_dim;
    reg         layer_type;
    reg  [`MULT_WIDTH-1:0]      rq_multiplier;
    reg  [`SHIFT_WIDTH-1:0]     rq_shift;
    reg  signed [`DATA_WIDTH-1:0] rq_zero_point;
    reg         compute_start;

    wire        compute_done;
    wire signed [`DATA_WIDTH-1:0] out_data;
    wire        out_valid;
    wire        out_last;

    // ========================================================================
    // DUT instantiation
    // ========================================================================
    cnn_core #(
        .DATA_WIDTH      (`DATA_WIDTH),
        .ACC_WIDTH       (`ACC_WIDTH),
        .PE_ROWS         (`PE_ROWS),
        .PE_COLS         (`PE_COLS),
        .MAX_K           (`MAX_KERNEL_SIZE),
        .MAX_DIM         (`MAX_IMG_DIM),
        .MAX_CH          (`MAX_CHANNELS),
        .BUF_DEPTH_IFMAP (65536),
        .BUF_AW_IFMAP    (16),
        .BUF_DEPTH_OFMAP (262144),
        .BUF_AW_OFMAP    (18),
        .WB_DEPTH        (65536),
        .WB_AW           (16)
    ) dut (
        .clk               (clk),
        .rst               (rst),
        .in_channels       (in_channels),
        .out_channels      (out_channels),
        .kernel_size       (kernel_size),
        .stride            (stride),
        .pad_en            (pad_en),
        .act_mode          (act_mode),
        .pool_en           (pool_en),
        .img_dim           (img_dim),
        .layer_type        (layer_type),
        .rq_multiplier     (rq_multiplier),
        .rq_shift          (rq_shift),
        .rq_zero_point     (rq_zero_point),
        .compute_start     (compute_start),
        .compute_done      (compute_done),
        .wt_ld_en          (1'b0),
        .wt_ld_addr        (16'd0),
        .wt_ld_data        (8'd0),
        .wt_swap           (1'b0),
        .ifmap_ext_wr_en   (1'b0),
        .ifmap_ext_wr_addr (16'd0),
        .ifmap_ext_wr_data (8'd0),
        .out_data          (out_data),
        .out_valid         (out_valid),
        .out_last          (out_last)
    );

    // ========================================================================
    // Golden output reference
    // ========================================================================
    reg [7:0] golden [0:OUT_TOTAL-1];   // [oy][ox][oc] order

    // ========================================================================
    // RTL output capture
    // ========================================================================
    reg [7:0]  rtl_out  [0:OUT_TOTAL-1];
    integer    out_idx;

    always @(posedge clk) begin
        if (rst) begin
            out_idx <= 0;
        end else if (out_valid && out_idx < OUT_TOTAL) begin
            rtl_out[out_idx] <= out_data;
            out_idx <= out_idx + 1;
        end
    end

    // ========================================================================
    // Scoreboard
    // ========================================================================
    integer total_mismatches;
    integer i_cmp;

    // ========================================================================
    // Main test sequence
    // ========================================================================
    initial begin
        $display("========================================================");
        $display("  tb_layer0 — Layer 0 Verification (Phase 9)");
        $display("  Conv 3->16, 3x3, pad=1, stride=1, LeakyReLU");
        $display("  Expected output: 262,144 INT8 values in [oy][ox][oc] order");
        $display("========================================================");

        // ── Reset ──────────────────────────────────────────────────────────
        rst           = 1;
        compute_start = 0;
        in_channels   = IN_CH;
        out_channels  = OUT_CH;
        kernel_size   = K_SIZE;
        stride        = 1'b0;       // stride-1
        pad_en        = 1'b1;       // padding enabled
        act_mode      = `ACT_LEAKY_RELU;
        pool_en       = 1'b0;       // no pooling
        img_dim       = IMG_DIM;
        layer_type    = `LAYER_TYPE_CONV;
        rq_multiplier = 16'd40370;  // From layer0_meta.json
        rq_shift      = 6'd23;
        rq_zero_point = 8'sd0;

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ── Load golden reference ──────────────────────────────────────────
        $display("[TB] Loading golden reference: rtl_hex/layer0_expected_rtl_order.hex");
        $readmemh("rtl_hex/layer0_expected_rtl_order.hex", golden);
        $display("[TB] Golden loaded: %0d entries", OUT_TOTAL);

        // ── Load weights (tiled format) into weight_buffer bank_a ──────────
        $display("[TB] Loading weights: rtl_hex/layer0_weights.hex (tiled, 432 bytes)");
        $readmemh("rtl_hex/layer0_weights.hex", dut.u_weight_buf.bank_a);
        $display("[TB] Weights loaded.");

        // ── Load biases into bias_rom ──────────────────────────────────────
        $display("[TB] Loading biases: rtl_hex/layer0_bias.hex (16 × INT32)");
        $readmemh("rtl_hex/layer0_bias.hex", dut.bias_rom, 0, 15);
        $display("[TB] Biases loaded.");

        // ── Load input feature map into ifmap_buffer.mem ───────────────────
        $display("[TB] Loading input: rtl_hex/layer0_input.hex (49,152 INT8 bytes)");
        $readmemh("rtl_hex/layer0_input.hex", dut.u_ifmap_buf.mem);
        $display("[TB] Input loaded.");

        repeat (5) @(posedge clk);

        // ── Start computation ──────────────────────────────────────────────
        $display("[TB] Asserting compute_start ...");
        @(posedge clk);
        compute_start = 1;
        @(posedge clk);
        compute_start = 0;

        $display("[TB] Compute started. Waiting for compute_done ...");
        $display("[TB] (Expected ~8.8M cycles at 100 MHz = ~88 ms simulation)");

        // ── Wait for completion (timeout = 15M cycles) ─────────────────────
        fork
            begin
                wait (compute_done == 1'b1);
                @(posedge clk);  // consume the compute_done pulse
                // Wait extra cycles so the last pipeline outputs are captured
                repeat (10) @(posedge clk);
                $display("[TB] compute_done received at time %0t ns", $time);
            end
            begin
                repeat (15_000_000) @(posedge clk);
                $display("[TB] TIMEOUT: compute_done not seen after 15M cycles.");
                $display("[TB] out_idx captured so far: %0d / %0d", out_idx, OUT_TOTAL);
                $finish;
            end
        join_any
        disable fork;

        // ── Report capture count ───────────────────────────────────────────
        $display("[TB] Output values captured: %0d / %0d", out_idx, OUT_TOTAL);
        if (out_idx != OUT_TOTAL) begin
            $display("[TB] WARNING: Unexpected output count. Expected %0d, got %0d.",
                     OUT_TOTAL, out_idx);
        end

        // ── Compare vs golden ──────────────────────────────────────────────
        $display("[TB] Comparing RTL output vs golden ...");
        total_mismatches = 0;

        for (i_cmp = 0; i_cmp < OUT_TOTAL; i_cmp = i_cmp + 1) begin
            if (rtl_out[i_cmp] !== golden[i_cmp]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 20) begin
                    // Decode position: i_cmp = oy*128*16 + ox*16 + oc
                    $display("  Mismatch[%6d] (oy=%3d ox=%3d oc=%2d): RTL=0x%02x  Golden=0x%02x",
                        i_cmp,
                        i_cmp / (IMG_DIM * OUT_CH),
                        (i_cmp / OUT_CH) % IMG_DIM,
                        i_cmp % OUT_CH,
                        rtl_out[i_cmp], golden[i_cmp]);
                end
            end
        end

        // ── Summary ────────────────────────────────────────────────────────
        $display("");
        $display("========================================================");
        $display("  RESULT SUMMARY");
        $display("========================================================");
        $display("  Total outputs captured : %0d / %0d", out_idx, OUT_TOTAL);
        $display("  Total mismatches       : %0d / %0d", total_mismatches, OUT_TOTAL);

        if (total_mismatches == 0 && out_idx == OUT_TOTAL)
            $display("  *** PASS: Layer 0 output matches Python golden model. ***");
        else
            $display("  *** FAIL: %0d mismatches detected. ***", total_mismatches);

        $display("========================================================");
        $finish;
    end

endmodule
