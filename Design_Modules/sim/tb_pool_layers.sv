/*
================================================================================
  tb_pool_layers.sv  —  Pool Layer Verification Testbench  (Phase 8)

  Tests 5 stride-2 pool layers (L1, L3, L5, L7, L9) of Tiny-YOLO in sequence.
  L11 (stride=1 darknet) is deferred — not yet supported in RTL.

  For each pool layer:
    Input  : layerN-1_output_chyx.hex  — [ch][y][x] order in ifmap_buffer
    Golden : layerN+1_input.hex        — [ch][y_out][x_out] order (same as stream)

  Pool output stream order: channel-first planar [ch][y_out][x_out], matching
  the channel-major ifmap layout expected by the next conv layer.

  DUT instantiated with large ifmap buffer (262144) to accommodate L1 which
  needs 16×128×128 = 262,144 bytes.  Ofmap = 65536 (max pool output = 65536).
================================================================================
*/

`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_pool_layers;

    // ========================================================================
    // Clock and reset
    // ========================================================================
    localparam CLK_PERIOD = 10;
    reg clk, rst;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // ========================================================================
    // DUT signals
    // ========================================================================
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

    // ========================================================================
    // DUT: large ifmap buffer (262144) to hold L1 input (16×128×128)
    // ========================================================================
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
        .BUF_DEPTH_OFMAP (65536),
        .BUF_AW_OFMAP    (16),
        .WB_DEPTH        (4096),
        .WB_AW           (12)
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
        .wt_ld_addr        (12'd0),
        .wt_ld_data        (8'd0),
        .wt_swap           (1'b0),
        .ifmap_ext_wr_en   (1'b0),
        .ifmap_ext_wr_addr (18'd0),
        .ifmap_ext_wr_data (8'd0),
        .out_data          (out_data),
        .out_valid         (out_valid),
        .out_last          (out_last)
    );

    // ========================================================================
    // Output capture — max pool output is L1: 16×64×64 = 65536
    // ========================================================================
    localparam integer OUT_MAX = 65536;
    reg [7:0] rtl_out [0:OUT_MAX-1];
    reg [7:0] golden  [0:OUT_MAX-1];
    integer   out_idx;
    integer   exp_total;

    always @(posedge clk) begin
        if (rst)
            out_idx <= 0;
        else if (out_valid && out_idx < exp_total)
            rtl_out[out_idx] <= out_data;
        // Increment unconditionally when valid so we catch overflow
        if (out_valid && !rst)
            out_idx <= out_idx + 1;
    end

    // ========================================================================
    // Task: run one pool layer
    // ========================================================================
    integer total_mismatches;
    integer i_cmp;

    task run_pool_layer;
        input integer p_in_ch;
        input [7:0]   p_dim;
        input integer p_out_total;  // expected # of pool outputs
        input [63:0]  p_layer_tag;  // display label (8-char ASCII)
        input [4095:0] p_inp_file;  // path to ifmap hex ([ch][y][x] format)
        input [4095:0] p_gold_file; // path to golden hex (same [ch][y_out][x_out])
    begin : pool_task
        integer timeout_cnt;

        // -- Reset -----------------------------------------------------------
        rst           = 1;
        compute_start = 0;
        in_channels   = p_in_ch[8:0];
        out_channels  = p_in_ch[8:0];  // pool out_ch == in_ch
        kernel_size   = 2'd2;
        stride        = 1'b1;           // stride-2
        pad_en        = 1'b0;
        act_mode      = `ACT_NONE;
        pool_en       = 1'b1;
        img_dim       = p_dim;
        layer_type    = `LAYER_TYPE_POOL;
        rq_multiplier = 16'd1;
        rq_shift      = 6'd0;
        rq_zero_point = 8'sd0;
        exp_total     = p_out_total;

        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // -- Load input and golden -------------------------------------------
        $display("[%s]  in_ch=%0d  dim=%0d  expected_out=%0d",
                 p_layer_tag, p_in_ch, p_dim, p_out_total);
        $readmemh(p_gold_file, golden);
        $readmemh(p_inp_file,  dut.u_ifmap_buf.mem);

        repeat (5) @(posedge clk);

        // -- Start -----------------------------------------------------------
        @(posedge clk);
        compute_start = 1;
        @(posedge clk);
        compute_start = 0;

        // -- Wait for compute_done (timeout = 2M cycles) ---------------------
        timeout_cnt = 0;
        fork
            begin
                wait (compute_done == 1'b1);
                @(posedge clk);
                repeat (10) @(posedge clk);
            end
            begin : timeout_blk
                repeat (2_000_000) @(posedge clk);
                $display("  TIMEOUT after 2M cycles.  out_idx=%0d", out_idx);
                disable timeout_blk;
            end
        join_any
        disable fork;

        // -- Compare ---------------------------------------------------------
        $display("  Captured: %0d / %0d", out_idx, exp_total);
        total_mismatches = 0;
        for (i_cmp = 0; i_cmp < exp_total; i_cmp = i_cmp + 1) begin
            if (rtl_out[i_cmp] !== golden[i_cmp]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 20) begin
                    $display("  Mismatch[%6d]: RTL=0x%02x  Golden=0x%02x",
                             i_cmp, rtl_out[i_cmp], golden[i_cmp]);
                end
            end
        end

        if (total_mismatches == 0 && out_idx == exp_total)
            $display("  *** PASS (%0d outputs, 0 mismatches) ***", exp_total);
        else
            $display("  *** FAIL: %0d mismatches, captured=%0d ***",
                     total_mismatches, out_idx);
        $display("--------------------------------------------------");
    end
    endtask

    // ========================================================================
    // Main sequence
    // ========================================================================
    initial begin
        $display("==========================================================");
        $display("  tb_pool_layers -- 5 Stride-2 Pool Layers (Phase 8)");
        $display("  L1, L3, L5, L7, L9  (L11 stride-1 deferred)");
        $display("==========================================================");

        // L1: pool after L0 — 16 ch, 128→64
        run_pool_layer(16, 128, 65536, "L1",
            "rtl_hex/layer0_output_chyx.hex",
            "rtl_hex/layer2_input.hex");

        // L3: pool after L2 — 32 ch, 64→32
        run_pool_layer(32, 64, 32768, "L3",
            "rtl_hex/layer2_output_chyx.hex",
            "rtl_hex/layer4_input.hex");

        // L5: pool after L4 — 64 ch, 32→16
        run_pool_layer(64, 32, 16384, "L5",
            "rtl_hex/layer4_output_chyx.hex",
            "rtl_hex/layer6_input.hex");

        // L7: pool after L6 — 128 ch, 16→8
        run_pool_layer(128, 16, 8192, "L7",
            "rtl_hex/layer6_output_chyx.hex",
            "rtl_hex/layer8_input.hex");

        // L9: pool after L8 — 256 ch, 8→4
        run_pool_layer(256, 8, 4096, "L9",
            "rtl_hex/layer8_output_chyx.hex",
            "rtl_hex/layer10_input.hex");

        $display("==========================================================");
        $display("  tb_pool_layers complete");
        $display("==========================================================");
        $finish;
    end

endmodule
