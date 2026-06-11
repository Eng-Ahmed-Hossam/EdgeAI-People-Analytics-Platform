/*
================================================================================
  tb_chained.sv  —  TRUE End-to-End Hardware Layer Chaining

  Runs the ENTIRE network in ONE simulation.  Only layer0_input.hex originates
  from Python.  Every subsequent layer's IFMAP is produced by reordering the
  PREVIOUS layer's RTL output stream — NO Python intermediate activation tensors.

  Reorder (emulates the production DMA engine, done in-testbench):
    - CONV output (tile-major [oc_tile][y][x][row]) -> IFMAP (channel-major
      [ch][y][x]) :  ch = tile*PE_ROWS + row ; addr = ch*dim^2 + y*dim + x
    - POOL output (already channel-major [ch][y][x]) -> direct copy

  Weights and biases are static model parameters (Python export), loaded per
  layer — these are NOT intermediate activation tensors.

  L11 (stride-1 darknet maxpool) is NOT implemented in cnn_core RTL.  It is
  bridged here by testbench reference logic (documented gap) so the chain can
  continue to the detection head.  17 of 18 layers execute through pure RTL.

  Final check: chained L18 RTL tensor vs Python golden layer18_expected_rtl_order.
================================================================================
*/
`timescale 1ns / 1ps
`include "edgeai_defs.vh"

module tb_chained;

    localparam CLK_PERIOD = 10;
    localparam PE = 16;
    reg clk, rst;
    initial clk = 0;
    always #(CLK_PERIOD/2) clk = ~clk;

    reg  [8:0]  in_channels, out_channels;
    reg  [1:0]  kernel_size;
    reg         stride, pad_en;
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
    wire        out_valid, out_last;

    cnn_core #(
        .DATA_WIDTH(`DATA_WIDTH), .ACC_WIDTH(`ACC_WIDTH),
        .PE_ROWS(`PE_ROWS), .PE_COLS(`PE_COLS),
        .MAX_K(`MAX_KERNEL_SIZE), .MAX_DIM(`MAX_IMG_DIM), .MAX_CH(`MAX_CHANNELS),
        .BUF_DEPTH_IFMAP(262144), .BUF_AW_IFMAP(18),
        .BUF_DEPTH_OFMAP(262144), .BUF_AW_OFMAP(18),
        .WB_DEPTH(1048576), .WB_AW(20)
    ) dut (
        .clk(clk), .rst(rst),
        .in_channels(in_channels), .out_channels(out_channels),
        .kernel_size(kernel_size), .stride(stride), .pad_en(pad_en),
        .act_mode(act_mode), .pool_en(pool_en), .img_dim(img_dim),
        .layer_type(layer_type),
        .rq_multiplier(rq_multiplier), .rq_shift(rq_shift), .rq_zero_point(rq_zero_point),
        .compute_start(compute_start), .compute_done(compute_done),
        .wt_ld_en(1'b0), .wt_ld_addr(20'd0), .wt_ld_data(8'd0), .wt_swap(1'b0),
        .ifmap_ext_wr_en(1'b0), .ifmap_ext_wr_addr(18'd0), .ifmap_ext_wr_data(8'd0),
        .out_data(out_data), .out_valid(out_valid), .out_last(out_last)
    );

    localparam integer CAP_MAX = 262144;
    reg [7:0] cap    [0:CAP_MAX-1];   // captured RTL output stream
    reg [7:0] golden [0:4095];        // L18 golden (4080)
    reg [7:0] tmp    [0:4095];        // scratch for L11 darknet pool
    integer   exp_total, out_idx;

    always @(posedge clk) begin
        if (rst) out_idx <= 0;
        else if (out_valid && out_idx < exp_total) begin
            cap[out_idx] <= out_data;
            out_idx <= out_idx + 1;
        end
    end

    integer i, t, y, x, r, ch, base, active, ntile, dimsq;
    integer m0, m1, m2, m3, mx, yy, xx;

    task do_reset; begin
        rst = 1; compute_start = 0;
        repeat (12) @(posedge clk);
        rst = 0; repeat (4) @(posedge clk);
    end endtask

    // Run a single conv or pool layer; output stream lands in cap[0..exp_total-1]
    task run_layer;
        input integer p_in_ch, p_out_ch, p_kernel, p_dim, p_pad;
        input [1:0]   p_act;
        input integer p_rq_m, p_rq_sh, p_rq_zp, p_out_total, p_is_pool;
        input [255:0] tag;
    begin
        do_reset();
        in_channels=p_in_ch[8:0]; out_channels=p_out_ch[8:0];
        kernel_size=p_kernel[1:0];
        pad_en=(p_pad!=0); act_mode=p_act; img_dim=p_dim[7:0];
        rq_multiplier=p_rq_m[15:0]; rq_shift=p_rq_sh[5:0]; rq_zero_point=p_rq_zp[7:0];
        exp_total=p_out_total;
        if (p_is_pool) begin
            layer_type=`LAYER_TYPE_POOL; pool_en=1'b1; stride=1'b1; kernel_size=2'd2;
        end else begin
            layer_type=`LAYER_TYPE_CONV; pool_en=1'b0; stride=1'b0;
        end
        repeat (4) @(posedge clk);
        @(posedge clk); compute_start=1; @(posedge clk); compute_start=0;
        fork
            begin wait(compute_done==1'b1); @(posedge clk); repeat (8) @(posedge clk); end
            begin repeat (40_000_000) @(posedge clk);
                  $display("  [%0s] TIMEOUT out_idx=%0d/%0d", tag, out_idx, p_out_total); end
        join_any
        disable fork;
        $display("  [%0s] streamed %0d / %0d outputs", tag, out_idx, p_out_total);
    end endtask

    // Reorder captured CONV stream (tile-major) -> ifmap buffer (channel-major)
    task reorder_conv_to_ifmap;
        input integer C, dim;
    begin
        dimsq = dim*dim;
        ntile = (C + PE - 1)/PE;
        i = 0;
        for (t=0; t<ntile; t=t+1) begin
            base = t*PE;
            active = (PE < (C-base)) ? PE : (C-base);
            for (y=0; y<dim; y=y+1)
              for (x=0; x<dim; x=x+1)
                for (r=0; r<active; r=r+1) begin
                    ch = base + r;
                    dut.u_ifmap_buf.mem[ch*dimsq + y*dim + x] = cap[i];
                    i = i + 1;
                end
        end
    end endtask

    // Pool stream is already channel-major -> direct copy
    task copy_to_ifmap;
        input integer N;
    begin
        for (i=0; i<N; i=i+1) dut.u_ifmap_buf.mem[i] = cap[i];
    end endtask

    // L11 stride-1 darknet maxpool (2x2, right/bottom pad = INT8_MIN), 256ch dim4->4.
    // Reads current ifmap (already holding L10 reordered output), writes result back.
    // Documented bridge for the one RTL-unimplemented layer.
    task l11_darknet_pool;
        integer c;
    begin
        dimsq = 16; // 4*4
        for (c=0; c<256; c=c+1) begin
            for (y=0; y<4; y=y+1)
              for (x=0; x<4; x=x+1) begin
                  // current value
                  m0 = $signed(dut.u_ifmap_buf.mem[c*16 + y*4 + x]);
                  // right neighbor (pad -128 if x+1>3)
                  m1 = (x+1<=3) ? $signed(dut.u_ifmap_buf.mem[c*16 + y*4 + (x+1)]) : -128;
                  // bottom neighbor
                  m2 = (y+1<=3) ? $signed(dut.u_ifmap_buf.mem[c*16 + (y+1)*4 + x]) : -128;
                  // bottom-right
                  m3 = (x+1<=3 && y+1<=3) ? $signed(dut.u_ifmap_buf.mem[c*16 + (y+1)*4 + (x+1)]) : -128;
                  mx = m0;
                  if (m1>mx) mx=m1;
                  if (m2>mx) mx=m2;
                  if (m3>mx) mx=m3;
                  tmp[c*16 + y*4 + x] = mx[7:0];
              end
        end
        for (i=0; i<4096; i=i+1) dut.u_ifmap_buf.mem[i] = tmp[i];
    end endtask

    integer mism, maxe, sume, d, ad;

    initial begin
        $display("======================================================");
        $display("  tb_chained -- TRUE end-to-end RTL layer chaining");
        $display("  Only L0 input from Python; all else from RTL outputs");
        $display("======================================================");

        rst=1; compute_start=0;
        in_channels=0; out_channels=0; kernel_size=0; stride=0; pad_en=0;
        act_mode=0; pool_en=0; img_dim=0; layer_type=0;
        rq_multiplier=0; rq_shift=0; rq_zero_point=0; exp_total=0;
        repeat (15) @(posedge clk); rst=0; repeat(4) @(posedge clk);

        // ---- Seed: ONLY Python-origin tensor in the whole run ----
        $readmemh("rtl_hex/layer0_input.hex", dut.u_ifmap_buf.mem);

        // ===== L0 CONV 3->16 =====
        $readmemh("rtl_hex/layer0_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer0_bias.hex", dut.bias_rom, 0, 15);
        run_layer(3,16,3,128,1,`ACT_LEAKY_RELU,40370,23,0,262144,0,"L0");
        reorder_conv_to_ifmap(16,128);

        // ===== L1 POOL 16ch 128->64 =====
        run_layer(16,16,2,128,0,`ACT_NONE,1,0,0,65536,1,"L1");
        copy_to_ifmap(65536);

        // ===== L2 CONV 16->32 =====
        $readmemh("rtl_hex/layer2_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer2_bias.hex", dut.bias_rom, 0, 31);
        run_layer(16,32,3,64,1,`ACT_LEAKY_RELU,50412,24,0,131072,0,"L2");
        reorder_conv_to_ifmap(32,64);

        // ===== L3 POOL 32ch 64->32 =====
        run_layer(32,32,2,64,0,`ACT_NONE,1,0,0,32768,1,"L3");
        copy_to_ifmap(32768);

        // ===== L4 CONV 32->64 =====
        $readmemh("rtl_hex/layer4_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer4_bias.hex", dut.bias_rom, 0, 63);
        run_layer(32,64,3,32,1,`ACT_LEAKY_RELU,36616,24,0,65536,0,"L4");
        reorder_conv_to_ifmap(64,32);

        // ===== L5 POOL 64ch 32->16 =====
        run_layer(64,64,2,32,0,`ACT_NONE,1,0,0,16384,1,"L5");
        copy_to_ifmap(16384);

        // ===== L6 CONV 64->128 =====
        $readmemh("rtl_hex/layer6_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer6_bias.hex", dut.bias_rom, 0, 127);
        run_layer(64,128,3,16,1,`ACT_LEAKY_RELU,51468,25,0,32768,0,"L6");
        reorder_conv_to_ifmap(128,16);

        // ===== L7 POOL 128ch 16->8 =====
        run_layer(128,128,2,16,0,`ACT_NONE,1,0,0,8192,1,"L7");
        copy_to_ifmap(8192);

        // ===== L8 CONV 128->256 =====
        $readmemh("rtl_hex/layer8_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer8_bias.hex", dut.bias_rom, 0, 255);
        run_layer(128,256,3,8,1,`ACT_LEAKY_RELU,39603,25,0,16384,0,"L8");
        reorder_conv_to_ifmap(256,8);

        // ===== L9 POOL 256ch 8->4 =====
        run_layer(256,256,2,8,0,`ACT_NONE,1,0,0,4096,1,"L9");
        copy_to_ifmap(4096);

        // ===== L10 CONV 256->256 =====
        $readmemh("rtl_hex/layer10_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer10_bias.hex", dut.bias_rom, 0, 255);
        run_layer(256,256,3,4,1,`ACT_LEAKY_RELU,54718,26,0,4096,0,"L10");
        reorder_conv_to_ifmap(256,4);

        // ===== L11 stride-1 darknet pool (testbench-bridged RTL gap) =====
        $display("  [L11] stride-1 darknet maxpool emulated in-testbench (RTL gap)");
        l11_darknet_pool();

        // ===== L12 CONV 256->256 =====
        $readmemh("rtl_hex/layer12_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer12_bias.hex", dut.bias_rom, 0, 255);
        run_layer(256,256,3,4,1,`ACT_LEAKY_RELU,56779,26,0,4096,0,"L12");
        reorder_conv_to_ifmap(256,4);

        // ===== L13 CONV 256->256 1x1 =====
        $readmemh("rtl_hex/layer13_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer13_bias.hex", dut.bias_rom, 0, 255);
        run_layer(256,256,1,4,0,`ACT_LEAKY_RELU,51601,26,0,4096,0,"L13");
        reorder_conv_to_ifmap(256,4);

        // ===== L14 CONV 256->256 3x3 =====
        $readmemh("rtl_hex/layer14_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer14_bias.hex", dut.bias_rom, 0, 255);
        run_layer(256,256,3,4,1,`ACT_LEAKY_RELU,62723,26,0,4096,0,"L14");
        reorder_conv_to_ifmap(256,4);

        // ===== L15 CONV 256->255 1x1 (det head #1, LINEAR) =====
        $readmemh("rtl_hex/layer15_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer15_bias.hex", dut.bias_rom, 0, 254);
        run_layer(256,255,1,4,0,`ACT_NONE,41974,25,0,4080,0,"L15");
        // L15 is a detection head output (branch); main path continues from L14->L16?
        // Per topology L16 consumes L15 output (255ch). Reorder L15 -> ifmap.
        reorder_conv_to_ifmap(255,4);

        // ===== L16 CONV 255->128 1x1 =====
        $readmemh("rtl_hex/layer16_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer16_bias.hex", dut.bias_rom, 0, 127);
        run_layer(255,128,1,4,0,`ACT_LEAKY_RELU,41738,25,0,2048,0,"L16");
        reorder_conv_to_ifmap(128,4);

        // ===== L17 CONV 128->256 3x3 =====
        $readmemh("rtl_hex/layer17_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer17_bias.hex", dut.bias_rom, 0, 255);
        run_layer(128,256,3,4,1,`ACT_LEAKY_RELU,22027,31,0,4096,0,"L17");
        reorder_conv_to_ifmap(256,4);

        // ===== L18 CONV 256->255 1x1 (det head #2, LINEAR) =====
        $readmemh("rtl_hex/layer18_weights.hex", dut.u_weight_buf.bank_a);
        $readmemh("rtl_hex/layer18_bias.hex", dut.bias_rom, 0, 254);
        run_layer(256,255,1,4,0,`ACT_NONE,62657,25,0,4080,0,"L18");

        // ---- Final compare: chained RTL L18 vs Python golden ----
        $readmemh("rtl_hex/layer18_expected_rtl_order.hex", golden, 0, 4079);
        mism=0; maxe=0; sume=0;
        for (i=0; i<4080; i=i+1) begin
            d = $signed(cap[i]) - $signed(golden[i]);
            ad = (d<0)?-d:d;
            sume = sume + ad;
            if (ad>maxe) maxe=ad;
            if (cap[i]!==golden[i]) begin
                mism=mism+1;
                if (mism<=10) $display("    L18 mism[%0d] RTL=%02h Golden=%02h", i, cap[i], golden[i]);
            end
        end
        // dump chained tensor
        begin integer fd; fd=$fopen("rtl_hex/layer18_chained_capture.hex","w");
            for (i=0;i<4080;i=i+1) $fwrite(fd,"%02x\n",cap[i]); $fclose(fd); end

        $display("======================================================");
        $display("  CHAINED FINAL TENSOR vs PYTHON GOLDEN");
        $display("    Size=%0d  Mismatches=%0d  MaxErr=%0d  MeanErr=%f",
                 4080, mism, maxe, (sume*1.0)/4080.0);
        if (mism==0)
            $display("  *** PASS: chained RTL tensor == Python golden (byte-exact) ***");
        else
            $display("  *** %0d / 4080 bytes differ ***", mism);
        $display("======================================================");
        $finish;
    end

endmodule
