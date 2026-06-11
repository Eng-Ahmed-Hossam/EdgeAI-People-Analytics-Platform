/*
================================================================================
  tb_signoff.sv  —  Final Sign-Off Verification (all 13 conv layers)

  Extends tb_conv_layers with per-layer NUMERICAL statistics:
    - Mismatch count
    - Max absolute error  (|rtl - golden| treated as signed INT8)
    - Mean absolute error
  And CAPTURES the L18 detection tensor stream to:
    rtl_hex/layer18_rtl_capture.hex   (tile-major stream order, == golden format)

  This is the authoritative sign-off run.  Each conv layer loads its
  Python-computed golden input and is compared byte-exact against the
  Python golden output in RTL stream order.
================================================================================
*/
`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_signoff;

    localparam CLK_PERIOD = 10;
    reg clk, rst;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

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

    localparam integer OUT_MAX = 262144;
    reg [7:0] rtl_out [0:OUT_MAX-1];
    reg [7:0] golden  [0:OUT_MAX-1];
    integer   exp_total;
    integer   out_idx;

    always @(posedge clk) begin
        if (rst) begin
            out_idx <= 0;
        end else if (out_valid && out_idx < exp_total) begin
            rtl_out[out_idx] <= out_data;
            out_idx <= out_idx + 1;
        end
    end

    integer total_pass;
    integer total_fail;
    integer total_mismatches;
    integer max_abs_err;
    integer sum_abs_err;
    integer ii;
    integer rtl_s, gold_s, diff, adiff;
    integer capture_fd;

    task do_reset;
    begin
        rst           = 1;
        compute_start = 0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (5)  @(posedge clk);
    end
    endtask

    // p_capture: when 1, dump rtl_out[] to layer18_rtl_capture.hex
    task run_conv_layer;
        input integer    p_in_ch;
        input integer    p_out_ch;
        input integer    p_kernel;
        input integer    p_dim;
        input integer    p_pad;
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
        input            p_capture;
    begin
        $display("--------------------------------------------------");
        $display("[%0s]  CONV in=%0d out=%0d k=%0dx%0d pad=%0d dim=%0d  rq=(M=%0d,s=%0d,zp=%0d)",
                 p_layer_tag, p_in_ch, p_out_ch, p_kernel, p_kernel, p_pad, p_dim,
                 p_rq_m, p_rq_sh, p_rq_zp);

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
                $display("  TIMEOUT after 30M cycles!  out_idx=%0d / %0d", out_idx, p_out_total);
                total_fail = total_fail + 1;
                disable fork;
            end
        join_any
        disable fork;

        // ---- Numerical statistics ----
        total_mismatches = 0;
        max_abs_err      = 0;
        sum_abs_err      = 0;
        for (ii = 0; ii < p_out_total; ii = ii + 1) begin
            // Interpret bytes as signed INT8
            rtl_s  = $signed(rtl_out[ii]);
            gold_s = $signed(golden[ii]);
            diff   = rtl_s - gold_s;
            adiff  = (diff < 0) ? -diff : diff;
            sum_abs_err = sum_abs_err + adiff;
            if (adiff > max_abs_err) max_abs_err = adiff;
            if (rtl_out[ii] !== golden[ii]) begin
                total_mismatches = total_mismatches + 1;
                if (total_mismatches <= 5)
                    $display("    Mismatch[%0d] RTL=%02h(%0d) Golden=%02h(%0d)",
                             ii, rtl_out[ii], rtl_s, golden[ii], gold_s);
            end
        end

        $display("  Expected=%0d  Captured=%0d  Mismatches=%0d  MaxAbsErr=%0d  MeanAbsErr=%f",
                 p_out_total, out_idx, total_mismatches, max_abs_err,
                 (sum_abs_err * 1.0) / (p_out_total * 1.0));

        if (total_mismatches == 0 && out_idx == p_out_total) begin
            $display("  RESULT: PASS  (0 mismatches, max_err=0, mean_err=0.0)");
            total_pass = total_pass + 1;
        end else begin
            $display("  RESULT: FAIL  mismatches=%0d captured=%0d/%0d",
                     total_mismatches, out_idx, p_out_total);
            total_fail = total_fail + 1;
        end

        // ---- Capture L18 detection tensor ----
        if (p_capture) begin
            capture_fd = $fopen("rtl_hex/layer18_rtl_capture.hex", "w");
            for (ii = 0; ii < p_out_total; ii = ii + 1)
                $fwrite(capture_fd, "%02x\n", rtl_out[ii]);
            $fclose(capture_fd);
            $display("  >> Captured L18 detection tensor (%0d bytes) -> rtl_hex/layer18_rtl_capture.hex",
                     p_out_total);
        end
    end
    endtask

    initial begin
        $display("==========================================================");
        $display("  tb_signoff -- Final Sign-Off (13 conv layers + L18 capture)");
        $display("==========================================================");

        total_pass = 0; total_fail = 0;
        rst = 1; compute_start = 0;
        in_channels=0; out_channels=0; kernel_size=0; stride=0;
        pad_en=0; act_mode=0; pool_en=0; img_dim=0; layer_type=0;
        rq_multiplier=0; rq_shift=0; rq_zero_point=0; exp_total=0;
        repeat (20) @(posedge clk);
        rst = 0;

        run_conv_layer(3,16,3,128,1,`ACT_LEAKY_RELU,40370,23,0,262144,"L0",
            "rtl_hex/layer0_weights.hex","rtl_hex/layer0_bias.hex",
            "rtl_hex/layer0_input.hex","rtl_hex/layer0_expected_rtl_order.hex",0);
        run_conv_layer(16,32,3,64,1,`ACT_LEAKY_RELU,50412,24,0,131072,"L2",
            "rtl_hex/layer2_weights.hex","rtl_hex/layer2_bias.hex",
            "rtl_hex/layer2_input.hex","rtl_hex/layer2_expected_rtl_order.hex",0);
        run_conv_layer(32,64,3,32,1,`ACT_LEAKY_RELU,36616,24,0,65536,"L4",
            "rtl_hex/layer4_weights.hex","rtl_hex/layer4_bias.hex",
            "rtl_hex/layer4_input.hex","rtl_hex/layer4_expected_rtl_order.hex",0);
        run_conv_layer(64,128,3,16,1,`ACT_LEAKY_RELU,51468,25,0,32768,"L6",
            "rtl_hex/layer6_weights.hex","rtl_hex/layer6_bias.hex",
            "rtl_hex/layer6_input.hex","rtl_hex/layer6_expected_rtl_order.hex",0);
        run_conv_layer(128,256,3,8,1,`ACT_LEAKY_RELU,39603,25,0,16384,"L8",
            "rtl_hex/layer8_weights.hex","rtl_hex/layer8_bias.hex",
            "rtl_hex/layer8_input.hex","rtl_hex/layer8_expected_rtl_order.hex",0);
        run_conv_layer(256,256,3,4,1,`ACT_LEAKY_RELU,54718,26,0,4096,"L10",
            "rtl_hex/layer10_weights.hex","rtl_hex/layer10_bias.hex",
            "rtl_hex/layer10_input.hex","rtl_hex/layer10_expected_rtl_order.hex",0);
        run_conv_layer(256,256,3,4,1,`ACT_LEAKY_RELU,56779,26,0,4096,"L12",
            "rtl_hex/layer12_weights.hex","rtl_hex/layer12_bias.hex",
            "rtl_hex/layer12_input.hex","rtl_hex/layer12_expected_rtl_order.hex",0);
        run_conv_layer(256,256,1,4,0,`ACT_LEAKY_RELU,51601,26,0,4096,"L13",
            "rtl_hex/layer13_weights.hex","rtl_hex/layer13_bias.hex",
            "rtl_hex/layer13_input.hex","rtl_hex/layer13_expected_rtl_order.hex",0);
        run_conv_layer(256,256,3,4,1,`ACT_LEAKY_RELU,62723,26,0,4096,"L14",
            "rtl_hex/layer14_weights.hex","rtl_hex/layer14_bias.hex",
            "rtl_hex/layer14_input.hex","rtl_hex/layer14_expected_rtl_order.hex",0);
        run_conv_layer(256,255,1,4,0,`ACT_NONE,41974,25,0,4080,"L15",
            "rtl_hex/layer15_weights.hex","rtl_hex/layer15_bias.hex",
            "rtl_hex/layer15_input.hex","rtl_hex/layer15_expected_rtl_order.hex",0);
        run_conv_layer(255,128,1,4,0,`ACT_LEAKY_RELU,41738,25,0,2048,"L16",
            "rtl_hex/layer16_weights.hex","rtl_hex/layer16_bias.hex",
            "rtl_hex/layer16_input.hex","rtl_hex/layer16_expected_rtl_order.hex",0);
        run_conv_layer(128,256,3,4,1,`ACT_LEAKY_RELU,22027,31,0,4096,"L17",
            "rtl_hex/layer17_weights.hex","rtl_hex/layer17_bias.hex",
            "rtl_hex/layer17_input.hex","rtl_hex/layer17_expected_rtl_order.hex",0);
        // L18 — detection head #2 — CAPTURE enabled
        run_conv_layer(256,255,1,4,0,`ACT_NONE,62657,25,0,4080,"L18",
            "rtl_hex/layer18_weights.hex","rtl_hex/layer18_bias.hex",
            "rtl_hex/layer18_input.hex","rtl_hex/layer18_expected_rtl_order.hex",1);

        $display("");
        $display("==========================================================");
        $display("  SIGN-OFF RESULT: %0d PASS / %0d FAIL  (of 13 conv layers)",
                 total_pass, total_fail);
        if (total_fail == 0)
            $display("  *** ALL 13 CONV LAYERS PASS: RTL == Python golden (byte-exact) ***");
        else
            $display("  *** FAILURES DETECTED ***");
        $display("==========================================================");
        $finish;
    end

endmodule
