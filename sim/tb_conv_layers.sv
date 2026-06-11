/*
================================================================================
  tb_conv_layers.sv  —  All 13 Conv Layer Verification (Phase 10)

  Tests all 13 conv layers of Tiny-YOLO in sequence using Python-computed
  inputs and bit-exact golden outputs from golden_network.py.

  DUT: cnn_core with WB_DEPTH=1048576 (20-bit) to handle large weight files
       (L10/L12/L14 each need 589,824 bytes).

  Method per layer:
    1. Assert reset, load weights / biases / ifmap via $readmemh
    2. Configure layer parameters (channels, kernel, pad, act, rq)
    3. Pulse compute_start; capture out_valid/out_data stream
    4. Compare vs golden; report PASS / FAIL

  Timeouts per layer (estimated cycles at 100 MHz):
    L0  ~8.8M    L6  ~22.5M   L13 ~1.3M
    L2  ~22.6M   L8  ~22.4M   L14 ~11.2M
    L4  ~22.5M   L10 ~11.2M   L15 ~1.3M
                 L12 ~11.2M   L16 ~0.6M
                              L17 ~5.6M
                              L18 ~1.3M
    Total: ~142M cycles ≈ 2-4 minutes wall time
================================================================================
*/
`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_conv_layers;

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
    reg  [`MULT_WIDTH-1:0]       rq_multiplier;
    reg  [`SHIFT_WIDTH-1:0]      rq_shift;
    reg  signed [`DATA_WIDTH-1:0] rq_zero_point;
    reg         compute_start;

    wire        compute_done;
    wire signed [`DATA_WIDTH-1:0] out_data;
    wire        out_valid;
    wire        out_last;

    // =========================================================================
    // DUT — wide weight buffer for deep layers (L10/L12/L14: 589,824 bytes)
    // =========================================================================
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
        .ifmap_ext_wr_addr (16'd0),
        .ifmap_ext_wr_data (8'd0),
        .out_data          (out_data),
        .out_valid         (out_valid),
        .out_last          (out_last)
    );

    // =========================================================================
    // Output capture (sized for largest layer: L0 = 262,144 entries)
    // =========================================================================
    localparam integer OUT_MAX = 262144;

    reg [7:0] rtl_out [0:OUT_MAX-1];
    reg [7:0] golden  [0:OUT_MAX-1];
    integer   exp_total;  // Set by test task before each layer
    integer   out_idx;

    always @(posedge clk) begin
        if (rst) begin
            out_idx <= 0;
        end else if (out_valid && out_idx < exp_total) begin
            rtl_out[out_idx] <= out_data;
            out_idx <= out_idx + 1;
        end
    end

    // =========================================================================
    // Scoreboard
    // =========================================================================
    integer total_mismatches;
    integer total_pass;
    integer total_fail;
    integer i_cmp;

    // =========================================================================
    // Task: reset DUT and wait for stability
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
    // Task: run one conv layer and report result
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
        input [255:0]    p_layer_tag; // display tag e.g. "L0"
        input [1023:0]   p_wt_file;
        input [1023:0]   p_bias_file;
        input [1023:0]   p_inp_file;
        input [1023:0]   p_gold_file;
        integer ii;
    begin
        $display("--------------------------------------------------");
        $display("[%0s]  in=%0d out=%0d  k=%0dx%0d  pad=%0d  dim=%0d",
                 p_layer_tag, p_in_ch, p_out_ch, p_kernel, p_kernel, p_pad, p_dim);

        // ── Reset ────────────────────────────────────────────────────────────
        do_reset();

        // ── Load memories ────────────────────────────────────────────────────
        $readmemh(p_wt_file,   dut.u_weight_buf.bank_a);
        $readmemh(p_bias_file, dut.bias_rom, 0, p_out_ch - 1);
        $readmemh(p_inp_file,  dut.u_ifmap_buf.mem);
        $readmemh(p_gold_file, golden, 0, p_out_total - 1);

        // ── Configure ────────────────────────────────────────────────────────
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

        // ── Start ────────────────────────────────────────────────────────────
        @(posedge clk);
        compute_start = 1;
        @(posedge clk);
        compute_start = 0;

        // ── Wait for compute_done (timeout = 30M cycles per layer) ───────────
        fork
            begin
                wait (compute_done == 1'b1);
                @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin
                repeat (30_000_000) @(posedge clk);
                $display("  TIMEOUT after 30M cycles!  out_idx=%0d / %0d", out_idx, p_out_total);
                total_fail = total_fail + 1;
                disable fork;
            end
        join_any
        disable fork;

        $display("  Captured: %0d / %0d", out_idx, p_out_total);

        // ── Compare ──────────────────────────────────────────────────────────
        total_mismatches = 0;
        for (ii = 0; ii < p_out_total; ii = ii + 1) begin
            if (rtl_out[ii] !== golden[ii]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 5) begin
                    // Decode position: i = oy*dim*out_ch + ox*out_ch + oc
                    $display("  Mismatch[%0d] RTL=%02h Golden=%02h", ii, rtl_out[ii], golden[ii]);
                end
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
    // Main test sequence
    // =========================================================================
    initial begin
        $display("==========================================================");
        $display("  tb_conv_layers — All 13 Conv Layers (Phase 10)");
        $display("==========================================================");

        total_pass = 0;
        total_fail = 0;
        rst           = 1;
        compute_start = 0;
        in_channels   = 0;  out_channels  = 0;
        kernel_size   = 0;  stride        = 0;
        pad_en        = 0;  act_mode      = 0;
        pool_en       = 0;  img_dim       = 0;
        layer_type    = 0;
        rq_multiplier = 0;  rq_shift      = 0;  rq_zero_point = 0;
        exp_total     = 0;
        repeat (20) @(posedge clk);
        rst = 0;

        // ── L0: Conv 3→16  3x3  pad=1  leaky  dim=128  (262,144 outputs) ────
        run_conv_layer(
            3, 16, 3, 128, 1, `ACT_LEAKY_RELU, 40370, 23, 0, 262144, "L0",
            "rtl_hex/layer0_weights.hex", "rtl_hex/layer0_bias.hex",
            "rtl_hex/layer0_input.hex",   "rtl_hex/layer0_expected_rtl_order.hex");

        // ── L2: Conv 16→32  3x3  pad=1  leaky  dim=64  (131,072 outputs) ────
        run_conv_layer(
            16, 32, 3, 64, 1, `ACT_LEAKY_RELU, 50412, 24, 0, 131072, "L2",
            "rtl_hex/layer2_weights.hex", "rtl_hex/layer2_bias.hex",
            "rtl_hex/layer2_input.hex",   "rtl_hex/layer2_expected_rtl_order.hex");

        // ── L4: Conv 32→64  3x3  pad=1  leaky  dim=32  (65,536 outputs) ─────
        run_conv_layer(
            32, 64, 3, 32, 1, `ACT_LEAKY_RELU, 36616, 24, 0, 65536, "L4",
            "rtl_hex/layer4_weights.hex", "rtl_hex/layer4_bias.hex",
            "rtl_hex/layer4_input.hex",   "rtl_hex/layer4_expected_rtl_order.hex");

        // ── L6: Conv 64→128  3x3  pad=1  leaky  dim=16  (32,768 outputs) ────
        run_conv_layer(
            64, 128, 3, 16, 1, `ACT_LEAKY_RELU, 51468, 25, 0, 32768, "L6",
            "rtl_hex/layer6_weights.hex", "rtl_hex/layer6_bias.hex",
            "rtl_hex/layer6_input.hex",   "rtl_hex/layer6_expected_rtl_order.hex");

        // ── L8: Conv 128→256  3x3  pad=1  leaky  dim=8  (16,384 outputs) ────
        run_conv_layer(
            128, 256, 3, 8, 1, `ACT_LEAKY_RELU, 39603, 25, 0, 16384, "L8",
            "rtl_hex/layer8_weights.hex", "rtl_hex/layer8_bias.hex",
            "rtl_hex/layer8_input.hex",   "rtl_hex/layer8_expected_rtl_order.hex");

        // ── L10: Conv 256→256  3x3  pad=1  leaky  dim=4  (4,096 outputs) ────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 54718, 26, 0, 4096, "L10",
            "rtl_hex/layer10_weights.hex", "rtl_hex/layer10_bias.hex",
            "rtl_hex/layer10_input.hex",   "rtl_hex/layer10_expected_rtl_order.hex");

        // ── L12: Conv 256→256  3x3  pad=1  leaky  dim=4  (4,096 outputs) ────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 56779, 26, 0, 4096, "L12",
            "rtl_hex/layer12_weights.hex", "rtl_hex/layer12_bias.hex",
            "rtl_hex/layer12_input.hex",   "rtl_hex/layer12_expected_rtl_order.hex");

        // ── L13: Conv 256→256  1x1  pad=0  leaky  dim=4  (4,096 outputs) ────
        run_conv_layer(
            256, 256, 1, 4, 0, `ACT_LEAKY_RELU, 51601, 26, 0, 4096, "L13",
            "rtl_hex/layer13_weights.hex", "rtl_hex/layer13_bias.hex",
            "rtl_hex/layer13_input.hex",   "rtl_hex/layer13_expected_rtl_order.hex");

        // ── L14: Conv 256→256  3x3  pad=1  leaky  dim=4  (4,096 outputs) ────
        run_conv_layer(
            256, 256, 3, 4, 1, `ACT_LEAKY_RELU, 62723, 26, 0, 4096, "L14",
            "rtl_hex/layer14_weights.hex", "rtl_hex/layer14_bias.hex",
            "rtl_hex/layer14_input.hex",   "rtl_hex/layer14_expected_rtl_order.hex");

        // ── L15: Conv 256→255  1x1  pad=0  linear  dim=4  (4,080 outputs) ───
        run_conv_layer(
            256, 255, 1, 4, 0, `ACT_NONE, 41974, 25, 0, 4080, "L15",
            "rtl_hex/layer15_weights.hex", "rtl_hex/layer15_bias.hex",
            "rtl_hex/layer15_input.hex",   "rtl_hex/layer15_expected_rtl_order.hex");

        // ── L16: Conv 255→128  1x1  pad=0  leaky  dim=4  (2,048 outputs) ────
        run_conv_layer(
            255, 128, 1, 4, 0, `ACT_LEAKY_RELU, 41738, 25, 0, 2048, "L16",
            "rtl_hex/layer16_weights.hex", "rtl_hex/layer16_bias.hex",
            "rtl_hex/layer16_input.hex",   "rtl_hex/layer16_expected_rtl_order.hex");

        // ── L17: Conv 128→256  3x3  pad=1  leaky  dim=4  (4,096 outputs) ────
        run_conv_layer(
            128, 256, 3, 4, 1, `ACT_LEAKY_RELU, 22027, 31, 0, 4096, "L17",
            "rtl_hex/layer17_weights.hex", "rtl_hex/layer17_bias.hex",
            "rtl_hex/layer17_input.hex",   "rtl_hex/layer17_expected_rtl_order.hex");

        // ── L18: Conv 256→255  1x1  pad=0  linear  dim=4  (4,080 outputs) ───
        run_conv_layer(
            256, 255, 1, 4, 0, `ACT_NONE, 62657, 25, 0, 4080, "L18",
            "rtl_hex/layer18_weights.hex", "rtl_hex/layer18_bias.hex",
            "rtl_hex/layer18_input.hex",   "rtl_hex/layer18_expected_rtl_order.hex");

        // ── Final summary ─────────────────────────────────────────────────────
        $display("");
        $display("==========================================================");
        $display("  FINAL RESULT: %0d PASS  /  %0d FAIL  (of 13 conv layers)",
                 total_pass, total_fail);
        if (total_fail == 0)
            $display("  *** ALL 13 CONV LAYERS PASS: RTL matches Python golden. ***");
        else
            $display("  *** FAILURES DETECTED — see per-layer output above. ***");
        $display("==========================================================");

        $finish;
    end

endmodule
