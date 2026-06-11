/*
================================================================================
  tb_full_network.sv  —  Phase 11: Full 19-Layer Network Verification

  Runs all 18 verifiable layers of Tiny-YOLO in sequence (L11 stride-1 pool
  is deferred — not yet implemented in RTL).

  13 conv layers: L0, L2, L4, L6, L8, L10, L12, L13, L14, L15, L16, L17, L18
   5 pool layers: L1, L3, L5, L7, L9
   1 skipped:    L11 (stride-1 darknet pool — not implemented)

  All inputs are Python golden hex files; outputs compared against golden.
  Layer chaining is validated independently per layer (each layer loads its
  pre-computed golden ifmap rather than the previous RTL output) — this
  verifies FSM switching between layer types and full parameter coverage.

  DUT parameters sized for the most demanding combination:
    BUF_DEPTH_IFMAP = 262144  (L1 pool: 16 * 128 * 128)
    BUF_DEPTH_OFMAP = 262144  (L0 conv output: 16 * 128 * 128)
    WB_DEPTH        = 1048576 (L10/L12/L14 weights: 589,824 bytes)
================================================================================
*/
`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_full_network;

    // =========================================================================
    // Clock
    // =========================================================================
    localparam CLK_PERIOD = 10;
    reg clk, rst;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    // DUT signals
    // =========================================================================
    reg  [8:0]  in_channels;
    reg  [8:0]  out_channels;
    reg  [1:0]  kernel_size;
    reg         stride;
    reg         pad_en;
    reg  [1:0]  act_mode;
    reg         pool_en;
    reg  [7:0]  img_dim;
    reg         layer_type;
    reg  [`MULT_WIDTH-1:0]        rq_multiplier;
    reg  [`SHIFT_WIDTH-1:0]       rq_shift;
    reg  signed [`DATA_WIDTH-1:0] rq_zero_point;
    reg         compute_start;

    wire        compute_done;
    wire signed [`DATA_WIDTH-1:0] out_data;
    wire        out_valid;
    wire        out_last;

    // =========================================================================
    // DUT — sized for full network
    // =========================================================================
    cnn_core #(
        .DATA_WIDTH      (`DATA_WIDTH),
        .ACC_WIDTH       (`ACC_WIDTH),
        .PE_ROWS         (`PE_ROWS),
        .PE_COLS         (`PE_COLS),
        .MAX_K           (`MAX_KERNEL_SIZE),
        .MAX_DIM         (`MAX_IMG_DIM),
        .MAX_CH          (`MAX_CHANNELS),
        .BUF_DEPTH_IFMAP (262144),
        .BUF_AW_IFMAP    (18),
        .BUF_DEPTH_OFMAP (262144),
        .BUF_AW_OFMAP    (18),
        .WB_DEPTH        (1048576),
        .WB_AW           (20)
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
        .wt_ld_addr        (20'd0),
        .wt_ld_data        (8'd0),
        .wt_swap           (1'b0),
        .ifmap_ext_wr_en   (1'b0),
        .ifmap_ext_wr_addr (18'd0),
        .ifmap_ext_wr_data (8'd0),
        .out_data          (out_data),
        .out_valid         (out_valid),
        .out_last          (out_last)
    );

    // =========================================================================
    // Output capture (max: L0 conv = 262,144 entries)
    // =========================================================================
    localparam integer OUT_MAX = 262144;
    reg [7:0] rtl_out [0:OUT_MAX-1];
    reg [7:0] golden  [0:OUT_MAX-1];
    integer   exp_total;
    integer   out_idx;

    always @(posedge clk) begin
        if (rst)
            out_idx <= 0;
        else if (out_valid && out_idx < exp_total) begin
            rtl_out[out_idx] <= out_data;
            out_idx          <= out_idx + 1;
        end
    end

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer total_pass;
    integer total_fail;
    integer total_mismatches;
    integer i_cmp;

    // =========================================================================
    // Task: reset DUT
    // =========================================================================
    task do_reset;
    begin
        rst           = 1;
        compute_start = 0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5)  @(posedge clk);
    end
    endtask

    // =========================================================================
    // Task: run one conv layer
    // =========================================================================
    task run_conv_layer;
        input integer    p_in_ch;
        input integer    p_out_ch;
        input integer    p_kernel;   // 1 or 3
        input integer    p_dim;
        input integer    p_pad;      // 0 or 1
        input [1:0]      p_act;
        input integer    p_rq_m;
        input integer    p_rq_sh;
        input integer    p_rq_zp;
        input integer    p_out_total;
        input [255:0]    p_layer_tag;
        input [1023:0]   p_wt_file;
        input [1023:0]   p_bias_file;
        input [1023:0]   p_inp_file;
        input [1023:0]   p_gold_file;
        integer ii;
    begin
        $display("--------------------------------------------------");
        $display("[%0s]  CONV  in=%0d out=%0d  k=%0dx%0d  pad=%0d  dim=%0d",
                 p_layer_tag, p_in_ch, p_out_ch, p_kernel, p_kernel, p_pad, p_dim);

        do_reset();

        $readmemh(p_wt_file,   dut.u_weight_buf.bank_a);
        $readmemh(p_bias_file, dut.bias_rom, 0, p_out_ch - 1);
        $readmemh(p_inp_file,  dut.u_ifmap_buf.mem);
        $readmemh(p_gold_file, golden, 0, p_out_total - 1);

        in_channels   = p_in_ch[8:0];
        out_channels  = p_out_ch[8:0];
        kernel_size   = p_kernel[1:0];
        stride        = 1'b0;
        pad_en        = (p_pad != 0) ? 1'b1 : 1'b0;
        act_mode      = p_act;
        pool_en       = 1'b0;
        img_dim       = p_dim[7:0];
        layer_type    = `LAYER_TYPE_CONV;
        rq_multiplier = p_rq_m[15:0];
        rq_shift      = p_rq_sh[5:0];
        rq_zero_point = p_rq_zp[7:0];
        exp_total     = p_out_total;

        repeat (5) @(posedge clk);

        @(posedge clk);
        compute_start = 1;
        @(posedge clk);
        compute_start = 0;

        fork
            begin
                wait (compute_done == 1'b1);
                @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin
                repeat (30_000_000) @(posedge clk);
                $display("  TIMEOUT after 30M cycles!  out_idx=%0d / %0d",
                         out_idx, p_out_total);
                total_fail = total_fail + 1;
                disable fork;
            end
        join_any
        disable fork;

        $display("  Captured: %0d / %0d", out_idx, p_out_total);

        total_mismatches = 0;
        for (ii = 0; ii < p_out_total; ii = ii + 1) begin
            if (rtl_out[ii] !== golden[ii]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 5)
                    $display("  Mismatch[%0d] RTL=%02h Golden=%02h",
                             ii, rtl_out[ii], golden[ii]);
            end
        end

        if (total_mismatches == 0 && out_idx == p_out_total) begin
            $display("  *** PASS (%0d outputs, 0 mismatches) ***", p_out_total);
            total_pass = total_pass + 1;
        end else begin
            $display("  *** FAIL  mismatches=%0d  captured=%0d/%0d ***",
                     total_mismatches, out_idx, p_out_total);
            total_fail = total_fail + 1;
        end
    end
    endtask

    // =========================================================================
    // Task: run one pool layer (stride-2, channel-sequential)
    // =========================================================================
    task run_pool_layer;
        input integer  p_in_ch;
        input [7:0]    p_dim;        // input spatial dimension
        input integer  p_out_total;  // expected output count
        input [255:0]  p_layer_tag;
        input [1023:0] p_inp_file;   // layerN_output_chyx.hex  (ch-major)
        input [1023:0] p_gold_file;  // layer{N+2}_input.hex    (ch-major)
        integer ii;
    begin
        $display("--------------------------------------------------");
        $display("[%0s]  POOL  in_ch=%0d  dim=%0d->%0d  expected=%0d",
                 p_layer_tag, p_in_ch, p_dim, p_dim >> 1, p_out_total);

        // Reset — out_idx clears via always block
        rst           = 1;
        compute_start = 0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        $readmemh(p_inp_file,  dut.u_ifmap_buf.mem);
        $readmemh(p_gold_file, golden, 0, p_out_total - 1);

        in_channels   = p_in_ch[8:0];
        out_channels  = p_in_ch[8:0];
        kernel_size   = 2'd2;
        stride        = 1'b1;
        pad_en        = 1'b0;
        act_mode      = `ACT_NONE;
        pool_en       = 1'b1;
        img_dim       = p_dim;
        layer_type    = `LAYER_TYPE_POOL;
        rq_multiplier = 16'd1;
        rq_shift      = 6'd0;
        rq_zero_point = 8'sd0;
        exp_total     = p_out_total;

        repeat (5) @(posedge clk);

        @(posedge clk);
        compute_start = 1;
        @(posedge clk);
        compute_start = 0;

        fork
            begin
                wait (compute_done == 1'b1);
                @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin
                repeat (2_000_000) @(posedge clk);
                $display("  TIMEOUT after 2M cycles!  out_idx=%0d / %0d",
                         out_idx, p_out_total);
                total_fail = total_fail + 1;
                disable fork;
            end
        join_any
        disable fork;

        $display("  Captured: %0d / %0d", out_idx, p_out_total);

        total_mismatches = 0;
        for (ii = 0; ii < p_out_total; ii = ii + 1) begin
            if (rtl_out[ii] !== golden[ii]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 5)
                    $display("  Mismatch[%0d] RTL=%02h Golden=%02h",
                             ii, rtl_out[ii], golden[ii]);
            end
        end

        if (total_mismatches == 0 && out_idx == p_out_total) begin
            $display("  *** PASS (%0d outputs, 0 mismatches) ***", p_out_total);
            total_pass = total_pass + 1;
        end else begin
            $display("  *** FAIL  mismatches=%0d  captured=%0d/%0d ***",
                     total_mismatches, out_idx, p_out_total);
            total_fail = total_fail + 1;
        end
    end
    endtask

    // =========================================================================
    // Main sequence: 18 layers (L11 skipped)
    // =========================================================================
    initial begin
        $display("==========================================================");
        $display("  tb_full_network -- Phase 11: Full 19-Layer Network");
        $display("  13 conv + 5 pool = 18 layers  (L11 stride-1 deferred)");
        $display("==========================================================");

        total_pass    = 0;
        total_fail    = 0;
        rst           = 1;
        compute_start = 0;
        in_channels   = 0; out_channels = 0;
        kernel_size   = 0; stride       = 0;
        pad_en        = 0; act_mode     = 0;
        pool_en       = 0; img_dim      = 0;
        layer_type    = 0;
        rq_multiplier = 0; rq_shift     = 0; rq_zero_point = 0;
        exp_total     = 0;
        repeat (20) @(posedge clk);
        rst = 0;

        // ── L0: Conv 3→16  3x3  pad=1  leaky  dim=128  262,144 out ──────────
        run_conv_layer(
            3, 16, 3, 128, 1, `ACT_LEAKY_RELU, 40370, 23, 0, 262144, "L0",
            "rtl_hex/layer0_weights.hex", "rtl_hex/layer0_bias.hex",
            "rtl_hex/layer0_input.hex",   "rtl_hex/layer0_expected_rtl_order.hex");

        // ── L1: Pool 16ch  128→64  65,536 out ────────────────────────────────
        run_pool_layer(16, 128, 65536, "L1",
            "rtl_hex/layer0_output_chyx.hex",
            "rtl_hex/layer2_input.hex");

        // ── L2: Conv 16→32  3x3  pad=1  leaky  dim=64  131,072 out ──────────
        run_conv_layer(
            16, 32, 3, 64, 1, `ACT_LEAKY_RELU, 50412, 24, 0, 131072, "L2",
            "rtl_hex/layer2_weights.hex", "rtl_hex/layer2_bias.hex",
            "rtl_hex/layer2_input.hex",   "rtl_hex/layer2_expected_rtl_order.hex");

        // ── L3: Pool 32ch  64→32  32,768 out ─────────────────────────────────
        run_pool_layer(32, 64, 32768, "L3",
            "rtl_hex/layer2_output_chyx.hex",
            "rtl_hex/layer4_input.hex");

        // ── L4: Conv 32→64  3x3  pad=1  leaky  dim=32  65,536 out ───────────
        run_conv_layer(
            32, 64, 3, 32, 1, `ACT_LEAKY_RELU, 36616, 24, 0, 65536, "L4",
            "rtl_hex/layer4_weights.hex", "rtl_hex/layer4_bias.hex",
            "rtl_hex/layer4_input.hex",   "rtl_hex/layer4_expected_rtl_order.hex");

        // ── L5: Pool 64ch  32→16  16,384 out ─────────────────────────────────
        run_pool_layer(64, 32, 16384, "L5",
            "rtl_hex/layer4_output_chyx.hex",
            "rtl_hex/layer6_input.hex");

        // ── L6: Conv 64→128  3x3  pad=1  leaky  dim=16  32,768 out ──────────
        run_conv_layer(
            64, 128, 3, 16, 1, `ACT_LEAKY_RELU, 51468, 25, 0, 32768, "L6",
            "rtl_hex/layer6_weights.hex", "rtl_hex/layer6_bias.hex",
            "rtl_hex/layer6_input.hex",   "rtl_hex/layer6_expected_rtl_order.hex");

        // ── L7: Pool 128ch  16→8  8,192 out ──────────────────────────────────
        run_pool_layer(128, 16, 8192, "L7",
            "rtl_hex/layer6_output_chyx.hex",
            "rtl_hex/layer8_input.hex");

        // ── L8: Conv 128→256  3x3  pad=1  leaky  dim=8  16,384 out ──────────
        run_conv_layer(
            128, 256, 3, 8, 1, `ACT_LEAKY_RELU, 39603, 25, 0, 16384, "L8",
            "rtl_hex/layer8_weights.hex", "rtl_hex/layer8_bias.hex",
            "rtl_hex/layer8_input.hex",   "rtl_hex/layer8_expected_rtl_order.hex");

        // ── L9: Pool 256ch  8→4  4,096 out ───────────────────────────────────
        run_pool_layer(256, 8, 4096, "L9",
            "rtl_hex/layer8_output_chyx.hex",
            "rtl_hex/layer10_input.hex");

        // ── L10: Conv 256→256  3x3  pad=1  leaky  dim=4  4,096 out ──────────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 54718, 26, 0, 4096, "L10",
            "rtl_hex/layer10_weights.hex", "rtl_hex/layer10_bias.hex",
            "rtl_hex/layer10_input.hex",   "rtl_hex/layer10_expected_rtl_order.hex");

        // ── L11: SKIPPED — stride-1 darknet pool not implemented ──────────────
        $display("--------------------------------------------------");
        $display("[L11] POOL stride-1 — SKIPPED (not implemented in RTL)");
        $display("  Using L12 golden input directly (bypasses L11).");
        $display("--------------------------------------------------");

        // ── L12: Conv 256→256  3x3  pad=1  leaky  dim=4  4,096 out ──────────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 56779, 26, 0, 4096, "L12",
            "rtl_hex/layer12_weights.hex", "rtl_hex/layer12_bias.hex",
            "rtl_hex/layer12_input.hex",   "rtl_hex/layer12_expected_rtl_order.hex");

        // ── L13: Conv 256→256  1x1  pad=0  leaky  dim=4  4,096 out ──────────
        run_conv_layer(
            256, 256, 1, 4, 0, `ACT_LEAKY_RELU, 51601, 26, 0, 4096, "L13",
            "rtl_hex/layer13_weights.hex", "rtl_hex/layer13_bias.hex",
            "rtl_hex/layer13_input.hex",   "rtl_hex/layer13_expected_rtl_order.hex");

        // ── L14: Conv 256→256  3x3  pad=1  leaky  dim=4  4,096 out ──────────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 62723, 26, 0, 4096, "L14",
            "rtl_hex/layer14_weights.hex", "rtl_hex/layer14_bias.hex",
            "rtl_hex/layer14_input.hex",   "rtl_hex/layer14_expected_rtl_order.hex");

        // ── L15: Conv 256→255  1x1  pad=0  linear  dim=4  4,080 out ─────────
        run_conv_layer(
            256, 255, 1, 4, 0, `ACT_NONE, 41974, 25, 0, 4080, "L15",
            "rtl_hex/layer15_weights.hex", "rtl_hex/layer15_bias.hex",
            "rtl_hex/layer15_input.hex",   "rtl_hex/layer15_expected_rtl_order.hex");

        // ── L16: Conv 255→128  1x1  pad=0  leaky  dim=4  2,048 out ──────────
        run_conv_layer(
            255, 128, 1, 4, 0, `ACT_LEAKY_RELU, 41738, 25, 0, 2048, "L16",
            "rtl_hex/layer16_weights.hex", "rtl_hex/layer16_bias.hex",
            "rtl_hex/layer16_input.hex",   "rtl_hex/layer16_expected_rtl_order.hex");

        // ── L17: Conv 128→256  3x3  pad=1  leaky  dim=4  4,096 out ──────────
        run_conv_layer(
            128, 256, 3, 4, 1, `ACT_LEAKY_RELU, 22027, 31, 0, 4096, "L17",
            "rtl_hex/layer17_weights.hex", "rtl_hex/layer17_bias.hex",
            "rtl_hex/layer17_input.hex",   "rtl_hex/layer17_expected_rtl_order.hex");

        // ── L18: Conv 256→255  1x1  pad=0  linear  dim=4  4,080 out ─────────
        run_conv_layer(
            256, 255, 1, 4, 0, `ACT_NONE, 62657, 25, 0, 4080, "L18",
            "rtl_hex/layer18_weights.hex", "rtl_hex/layer18_bias.hex",
            "rtl_hex/layer18_input.hex",   "rtl_hex/layer18_expected_rtl_order.hex");

        // ── Final summary ─────────────────────────────────────────────────────
        $display("");
        $display("==========================================================");
        $display("  FINAL RESULT: %0d PASS  /  %0d FAIL  (of 18 layers)",
                 total_pass, total_fail);
        $display("  (L11 stride-1 pool not counted — deferred)");
        if (total_fail == 0)
            $display("  *** ALL 18 LAYERS PASS: RTL matches Python golden. ***");
        else
            $display("  *** FAILURES DETECTED — see per-layer output above. ***");
        $display("==========================================================");

        $finish;
    end

endmodule
