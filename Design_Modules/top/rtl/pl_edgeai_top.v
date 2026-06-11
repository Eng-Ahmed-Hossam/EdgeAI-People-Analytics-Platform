/*
------------------------------------------------------------------------------
Module Name : pl_edgeai_top
Project     : EdgeAI Location Intelligence — Zynq Integration (v2.0)
Description :
    Top-level PL module for Zynq-7000 EdgeAI CNN accelerator.
    Replaces simulation-only SDRAM controller with AXI HP port stubs.

    System dataflow:
      OV7670 → camera_if → pixel_fifo → frame_writer → [HP0] → PS DDR3
      PS DDR3 → [HP1] → cnn_input_adapter → ifmap_buffer → cnn_core
      layer_scheduler → weight_dma [HP2] → cnn_core.wt_ld_*
      layer_scheduler → bias_dma   [HP2] → cnn_core.bias_ld_*
      cnn_core.out_stream → reorder_dma → ifmap_buffer (next layer)
      cnn_core.out_stream (L18) → tensor_output_buffer
      tensor_output_buffer → [AXI-Lite] → PS
      edgeai_ctrl_regs [GP0 AXI-Lite] → inference control, status

    AXI HP port assignments (to be connected in Vivado block design):
      HP0: frame_writer → DDR write
      HP1: frame_reader/cnn_input_adapter → DDR read
      HP2: weight_dma + bias_dma → DDR read (time-multiplexed)
      (HP3 optional: tensor DMA readback to PS DDR)

    NOTE: AXI master interfaces are exposed as flat wire bundles here.
    The Vivado IP packager or block design connection handles the
    HP port AXI slave connections.

    Clock domains:
      sys_clk (100 MHz, FCLK_CLK0): all PL RTL
      cam_pclk (~25 MHz, OV7670): camera_if, pixel_fifo write side

    Changes from v1.0:
      - sdram_ctrl removed; replaced by AXI HP wire bundles
      - xclk_gen removed; cam_xclk driven directly from FCLK_CLK1
      - sccb_ctrl removed; PS I2C0 via EMIO handles camera init
      - weight_dma, bias_dma, reorder_dma, edgeai_ctrl_regs added
      - cnn_core.bias_ld_* wired to bias_dma
      - reorder_dma routes conv output to ifmap_buffer (non-last layers)
      - L18 detection tensor routed directly to tensor_output_buffer
      - layer_scheduler.wt_load_done now driven by weight_dma
      - Real layer scheduling now waits for BOTH weight and bias loads

Change Log:
    v2.0 - Zynq integration: AXI HP, DMA modules, reorder_dma
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pl_edgeai_top #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter ACC_WIDTH  = `ACC_WIDTH,
    parameter PE_ROWS    = `PE_ROWS,
    parameter PE_COLS    = `PE_COLS
)(
    // ---- System ----
    input  wire        sys_clk,   // 100 MHz from FCLK_CLK0
    input  wire        sys_rst,

    // ---- OV7670 Camera DVP ----
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_d,
    input  wire        cam_xclk,  // Driven by FCLK_CLK1 externally (24 MHz)

    // ---- AXI HP0 Master (frame write to DDR) ----
    output wire [31:0] hp0_awaddr,
    output wire        hp0_awvalid,
    input  wire        hp0_awready,
    output wire [7:0]  hp0_awlen,
    output wire [2:0]  hp0_awsize,
    output wire [1:0]  hp0_awburst,
    output wire [31:0] hp0_wdata,
    output wire [3:0]  hp0_wstrb,
    output wire        hp0_wvalid,
    output wire        hp0_wlast,
    input  wire        hp0_wready,
    input  wire [1:0]  hp0_bresp,
    input  wire        hp0_bvalid,
    output wire        hp0_bready,

    // ---- AXI HP1 Master (frame read to CNN input adapter) ----
    output wire [31:0] hp1_araddr,
    output wire        hp1_arvalid,
    input  wire        hp1_arready,
    output wire [7:0]  hp1_arlen,
    output wire [2:0]  hp1_arsize,
    output wire [1:0]  hp1_arburst,
    input  wire [31:0] hp1_rdata,
    input  wire        hp1_rvalid,
    input  wire        hp1_rlast,
    output wire        hp1_rready,

    // ---- AXI HP2 Master (weight + bias DMA read from DDR) ----
    output wire [31:0] hp2_araddr,
    output wire        hp2_arvalid,
    input  wire        hp2_arready,
    output wire [7:0]  hp2_arlen,
    output wire [2:0]  hp2_arsize,
    output wire [1:0]  hp2_arburst,
    input  wire [31:0] hp2_rdata,
    input  wire        hp2_rvalid,
    input  wire        hp2_rlast,
    output wire        hp2_rready,

    // ---- AXI-Lite GP0 Slave (control registers, from PS) ----
    input  wire [31:0] gp0_awaddr,
    input  wire        gp0_awvalid,
    output wire        gp0_awready,
    input  wire [31:0] gp0_wdata,
    input  wire [3:0]  gp0_wstrb,
    input  wire        gp0_wvalid,
    output wire        gp0_wready,
    output wire [1:0]  gp0_bresp,
    output wire        gp0_bvalid,
    input  wire        gp0_bready,
    input  wire [31:0] gp0_araddr,
    input  wire        gp0_arvalid,
    output wire        gp0_arready,
    output wire [31:0] gp0_rdata,
    output wire [1:0]  gp0_rresp,
    output wire        gp0_rvalid,
    input  wire        gp0_rready,

    // ---- Status / Interrupt ----
    output wire        inference_done_irq,   // To PS IRQ_F2P
    output wire        cam_frame_error,
    output wire        tensor_ready,
    output wire        busy,

    // ---- Tensor read (PS reads tensor via AXI-Lite, or use DMA) ----
    input  wire        tensor_rd_en,
    input  wire [15:0] tensor_rd_addr,
    output wire signed [DATA_WIDTH-1:0] tensor_rd_data,
    output wire        tensor_rd_valid
);

    // =========================================================================
    // Control register outputs
    // =========================================================================
    wire        ctrl_inference_start;
    wire        ctrl_cnn_soft_reset;
    wire        ctrl_tensor_clear;
    wire [31:0] ctrl_wt_base;
    wire [31:0] ctrl_bias_base;
    wire [31:0] ctrl_frame_a;
    wire [31:0] ctrl_frame_b;
    wire [31:0] ctrl_tensor_dst;

    // =========================================================================
    // Internal wires
    // =========================================================================

    // Camera interface
    wire [15:0] cam_pixel_data;
    wire        cam_pixel_valid;
    wire        cam_frame_start;
    wire        cam_frame_end;

    // CDC synchronizer: cam_pclk → sys_clk
    reg [2:0] frame_start_sync;
    wire      frame_start_sysclk;
    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst) frame_start_sync <= 3'b0;
        else         frame_start_sync <= {frame_start_sync[1:0], cam_frame_start};
    end
    assign frame_start_sysclk = frame_start_sync[1] & ~frame_start_sync[2];

    // Pixel FIFO
    wire [15:0] fifo_rd_data;
    wire        fifo_empty;
    wire        fifo_full;
    wire        fifo_almost_full;
    wire        fw_fifo_rd_en;

    // Frame writer AXI HP0
    wire        fw_write_done;

    // Frame buffer manager
    wire [`SDRAM_ADDR_WIDTH-1:0] fbm_wr_base;   // Kept for compatibility
    wire [`SDRAM_ADDR_WIDTH-1:0] fbm_rd_base;
    wire        fbm_frame_ready;
    wire        fbm_buf_sel;

    // Resolved DDR frame addresses from control regs or fbm
    wire [31:0] ddr_frame_wr_base = fbm_buf_sel ? ctrl_frame_b : ctrl_frame_a;
    wire [31:0] ddr_frame_rd_base = fbm_buf_sel ? ctrl_frame_a : ctrl_frame_b;

    // CNN input adapter
    wire        adapter_start;
    wire        adapter_done;
    wire        adapter_ifmap_wr_en;
    wire [15:0] adapter_ifmap_wr_addr;
    wire signed [DATA_WIDTH-1:0] adapter_ifmap_wr_data;

    // Layer scheduler
    wire        sched_done;
    wire        sched_busy;
    wire [$clog2(`NUM_LAYERS)-1:0] sched_layer;
    wire [8:0]  sched_in_ch;
    wire [8:0]  sched_out_ch;
    wire [1:0]  sched_kernel;
    wire        sched_stride;
    wire        sched_pad;
    wire [1:0]  sched_act;
    wire        sched_pool;
    wire [7:0]  sched_dim;
    wire        sched_type;
    wire [`MULT_WIDTH-1:0]  sched_rq_mult;
    wire [`SHIFT_WIDTH-1:0] sched_rq_shift;
    wire signed [DATA_WIDTH-1:0] sched_rq_zp;
    wire        sched_wt_start;
    wire        sched_compute_start;
    wire        sched_buf_swap;

    // Weight DMA
    wire        wt_dma_done;
    wire        wt_ld_en;
    wire [19:0] wt_ld_addr;
    wire [7:0]  wt_ld_data;

    // Bias DMA
    wire        bias_dma_done;
    wire        bias_ld_en;
    wire [7:0]  bias_ld_addr;
    wire [31:0] bias_ld_data;

    // Both must complete before compute_start
    wire wt_load_done_hw = wt_dma_done & bias_dma_done;

    // HP2 arbiter: weight_dma has priority (larger transfer)
    // Simple grant: hp2 goes to weight_dma when it's active, else bias_dma
    wire wt_dma_arvalid, bias_dma_arvalid;
    wire [31:0] wt_dma_araddr, bias_dma_araddr;
    wire [7:0]  wt_dma_arlen,  bias_dma_arlen;
    wire [31:0] wt_dma_rdata,  bias_dma_rdata;
    wire        wt_dma_rvalid, bias_dma_rvalid;
    wire        wt_dma_rlast,  bias_dma_rlast;
    wire        wt_dma_rready, bias_dma_rready;
    wire        wt_dma_arready,bias_dma_arready;
    wire [2:0]  wt_dma_arsize, bias_dma_arsize;
    wire [1:0]  wt_dma_arburst,bias_dma_arburst;

    // HP2 mux: weight_dma gets bus when it requests; bias_dma otherwise
    assign hp2_araddr  = wt_dma_arvalid ? wt_dma_araddr  : bias_dma_araddr;
    assign hp2_arvalid = wt_dma_arvalid ? wt_dma_arvalid : bias_dma_arvalid;
    assign hp2_arlen   = wt_dma_arvalid ? wt_dma_arlen   : bias_dma_arlen;
    assign hp2_arsize  = wt_dma_arvalid ? wt_dma_arsize  : bias_dma_arsize;
    assign hp2_arburst = wt_dma_arvalid ? wt_dma_arburst : bias_dma_arburst;
    assign wt_dma_arready   = wt_dma_arvalid  ? hp2_arready : 1'b0;
    assign bias_dma_arready = !wt_dma_arvalid ? hp2_arready : 1'b0;
    assign wt_dma_rdata   = hp2_rdata;
    assign bias_dma_rdata = hp2_rdata;
    assign hp2_rready     = wt_dma_rready | bias_dma_rready;
    assign wt_dma_rvalid  = wt_dma_rready  ? hp2_rvalid : 1'b0;
    assign bias_dma_rvalid= bias_dma_rready ? hp2_rvalid : 1'b0;
    assign wt_dma_rlast   = hp2_rlast;
    assign bias_dma_rlast = hp2_rlast;

    // CNN core
    wire        core_compute_done;
    wire signed [DATA_WIDTH-1:0] core_out_data;
    wire        core_out_valid;
    wire        core_out_last;

    // Reorder DMA
    wire        reorder_done;
    wire        rdma_ifmap_wr_en;
    wire [17:0] rdma_ifmap_wr_addr;
    wire signed [DATA_WIDTH-1:0] rdma_ifmap_wr_data;

    // ifmap mux: adapter writes at layer-0 load; reorder writes between layers
    // adapter_done gates: adapter drives ifmap when active; reorder drives otherwise
    wire        ifmap_wr_en   = adapter_ifmap_wr_en | rdma_ifmap_wr_en;
    wire [17:0] ifmap_wr_addr = adapter_ifmap_wr_en ?
                                 {2'b0, adapter_ifmap_wr_addr} :
                                 rdma_ifmap_wr_addr;
    wire signed [DATA_WIDTH-1:0] ifmap_wr_data = adapter_ifmap_wr_en ?
                                                  adapter_ifmap_wr_data :
                                                  rdma_ifmap_wr_data;

    // Last layer detection
    wire is_last_layer = (sched_layer == 5'd18);

    // reorder_done clears after each layer; reorder_start = core_compute_done
    // (scatter phase begins immediately when compute finishes)

    // =========================================================================
    // Top-level control FSM
    // =========================================================================
    localparam TOP_IDLE       = 3'd0;
    localparam TOP_WAIT_FRAME = 3'd1;
    localparam TOP_LOAD_FRAME = 3'd2;
    localparam TOP_INFERENCE  = 3'd3;
    localparam TOP_DONE       = 3'd4;

    reg [2:0] top_state;
    reg       top_inference_done;

    assign busy            = (top_state != TOP_IDLE);
    assign inference_done_irq = top_inference_done;

    always @(posedge sys_clk) begin
        if (sys_rst || ctrl_cnn_soft_reset) begin
            top_state          <= TOP_IDLE;
            top_inference_done <= 1'b0;
        end else begin
            top_inference_done <= 1'b0;
            case (top_state)
                TOP_IDLE:       if (ctrl_inference_start) top_state <= TOP_WAIT_FRAME;
                TOP_WAIT_FRAME: if (fbm_frame_ready)      top_state <= TOP_LOAD_FRAME;
                TOP_LOAD_FRAME: if (adapter_done)         top_state <= TOP_INFERENCE;
                TOP_INFERENCE:  if (sched_done)           top_state <= TOP_DONE;
                TOP_DONE: begin
                    top_inference_done <= 1'b1;
                    top_state          <= TOP_IDLE;
                end
                default: top_state <= TOP_IDLE;
            endcase
        end
    end

    assign adapter_start = (top_state == TOP_WAIT_FRAME) && fbm_frame_ready;
    wire   sched_start   = (top_state == TOP_LOAD_FRAME) && adapter_done;

    // =========================================================================
    // Camera interface
    // =========================================================================
    camera_if #(
        .IMG_W (`IMG_WIDTH),
        .IMG_H (`IMG_HEIGHT)
    ) u_camera_if (
        .pclk       (cam_pclk),
        .rst        (sys_rst),
        .vsync      (cam_vsync),
        .href       (cam_href),
        .d          (cam_d),
        .pixel_data (cam_pixel_data),
        .pixel_valid(cam_pixel_valid),
        .frame_start(cam_frame_start),
        .frame_end  (cam_frame_end),
        .frame_error(cam_frame_error)
    );

    // =========================================================================
    // Pixel FIFO (CDC cam_pclk → sys_clk)
    // =========================================================================
    pixel_fifo u_pixel_fifo (
        .wr_clk     (cam_pclk),
        .wr_rst     (sys_rst),
        .wr_en      (cam_pixel_valid & ~fifo_full),
        .wr_data    (cam_pixel_data),
        .full       (fifo_full),
        .almost_full(fifo_almost_full),
        .rd_clk     (sys_clk),
        .rd_rst     (sys_rst),
        .rd_en      (fw_fifo_rd_en),
        .rd_data    (fifo_rd_data),
        .empty      (fifo_empty)
    );

    // =========================================================================
    // Frame writer — writes pixels from FIFO to PS DDR3 via AXI HP0
    // hp0_aw* and hp0_w* driven by frame_writer internal logic.
    // This is a stub placeholder for the AXI HP0 frame write path.
    // Full axi_hp_frame_writer.v is a separate deliverable for Vivado IP.
    // =========================================================================
    frame_writer u_frame_writer (
        .clk          (sys_clk),
        .rst          (sys_rst),
        .frame_start  (frame_start_sysclk),
        .base_addr    (fbm_wr_base),
        .write_done   (fw_write_done),
        .fifo_rd_en   (fw_fifo_rd_en),
        .fifo_data    (fifo_rd_data),
        .fifo_empty   (fifo_empty),
        // AXI HP0 write connections (wire through)
        .sdram_wr_req (),
        .sdram_wr_addr(),
        .sdram_wr_data(),
        .sdram_wr_ack (1'b1)   // Stub: always ack (replace with real HP0)
    );

    // HP0 write address — driven by frame_writer (full implementation in axi_hp_frame_writer.v)
    assign hp0_awaddr  = {8'd0, fbm_wr_base};
    assign hp0_awvalid = 1'b0;   // Placeholder
    assign hp0_awlen   = 8'd0;
    assign hp0_awsize  = 3'd2;
    assign hp0_awburst = 2'd1;
    assign hp0_wdata   = 32'd0;
    assign hp0_wstrb   = 4'd0;
    assign hp0_wvalid  = 1'b0;
    assign hp0_wlast   = 1'b0;
    assign hp0_bready  = 1'b1;

    // =========================================================================
    // Frame buffer manager
    // =========================================================================
    frame_buffer_mgr u_fbm (
        .clk          (sys_clk),
        .rst          (sys_rst),
        .wr_done      (fw_write_done),
        .wr_base_addr (fbm_wr_base),
        .rd_done      (sched_done),
        .rd_base_addr (fbm_rd_base),
        .frame_ready  (fbm_frame_ready),
        .buf_sel      (fbm_buf_sel)
    );

    // =========================================================================
    // CNN input adapter — reads frame from DDR via HP1, writes to ifmap_buffer
    // hp1_ar* driven by adapter (full implementation in axi_hp_frame_reader.v)
    // =========================================================================
    cnn_input_adapter u_adapter (
        .clk             (sys_clk),
        .rst             (sys_rst),
        .start           (adapter_start),
        .frame_base_addr (fbm_rd_base),
        .done            (adapter_done),
        // HP1 read stub (full axi_hp_frame_reader.v to be connected in Vivado)
        .sdram_rd_req    (),
        .sdram_rd_addr   (),
        .sdram_rd_data   (16'd0),   // Placeholder
        .sdram_rd_valid  (1'b0),
        .ifmap_wr_en     (adapter_ifmap_wr_en),
        .ifmap_wr_addr   (adapter_ifmap_wr_addr),
        .ifmap_wr_data   (adapter_ifmap_wr_data)
    );

    // HP1 placeholder
    assign hp1_araddr  = 32'd0;
    assign hp1_arvalid = 1'b0;
    assign hp1_arlen   = 8'd0;
    assign hp1_arsize  = 3'd2;
    assign hp1_arburst = 2'd1;
    assign hp1_rready  = 1'b0;

    // =========================================================================
    // Layer scheduler
    // =========================================================================
    layer_scheduler u_scheduler (
        .clk            (sys_clk),
        .rst            (sys_rst),
        .start          (sched_start),
        .done           (sched_done),
        .busy           (sched_busy),
        .current_layer  (sched_layer),
        .in_channels    (sched_in_ch),
        .out_channels   (sched_out_ch),
        .kernel_size    (sched_kernel),
        .stride         (sched_stride),
        .pad_en         (sched_pad),
        .act_mode       (sched_act),
        .pool_en        (sched_pool),
        .img_dim        (sched_dim),
        .layer_type     (sched_type),
        .rq_multiplier  (sched_rq_mult),
        .rq_shift       (sched_rq_shift),
        .rq_zero_point  (sched_rq_zp),
        .wt_load_start  (sched_wt_start),
        .wt_load_done   (wt_load_done_hw),   // Now connected to real DMA done
        .compute_start  (sched_compute_start),
        .compute_done   (core_compute_done),
        .buf_swap       (sched_buf_swap)
    );

    // =========================================================================
    // Weight DMA
    // =========================================================================
    weight_dma #(
        .AXI_AW (32),
        .AXI_DW (32),
        .WT_AW  (20)
    ) u_weight_dma (
        .clk           (sys_clk),
        .rst           (sys_rst),
        .layer_idx     (sched_layer),
        .ddr_base_addr (ctrl_wt_base),
        .load_start    (sched_wt_start),
        .load_done     (wt_dma_done),
        // AXI HP2 read (arbitrated)
        .m_axi_araddr  (wt_dma_araddr),
        .m_axi_arvalid (wt_dma_arvalid),
        .m_axi_arready (wt_dma_arready),
        .m_axi_arlen   (wt_dma_arlen),
        .m_axi_arsize  (wt_dma_arsize),
        .m_axi_arburst (wt_dma_arburst),
        .m_axi_rdata   (wt_dma_rdata),
        .m_axi_rvalid  (wt_dma_rvalid),
        .m_axi_rlast   (wt_dma_rlast),
        .m_axi_rready  (wt_dma_rready),
        // weight_buffer write
        .wt_ld_en      (wt_ld_en),
        .wt_ld_addr    (wt_ld_addr),
        .wt_ld_data    (wt_ld_data)
    );

    // =========================================================================
    // Bias DMA
    // =========================================================================
    bias_dma #(
        .AXI_AW  (32),
        .AXI_DW  (32),
        .BIAS_AW (8)
    ) u_bias_dma (
        .clk           (sys_clk),
        .rst           (sys_rst),
        .layer_idx     (sched_layer),
        .bias_base_addr(ctrl_bias_base),
        .load_start    (sched_wt_start),    // Same trigger as weight_dma
        .load_done     (bias_dma_done),
        // AXI HP2 read (arbitrated, lower priority)
        .m_axi_araddr  (bias_dma_araddr),
        .m_axi_arvalid (bias_dma_arvalid),
        .m_axi_arready (bias_dma_arready),
        .m_axi_arlen   (bias_dma_arlen),
        .m_axi_arsize  (bias_dma_arsize),
        .m_axi_arburst (bias_dma_arburst),
        .m_axi_rdata   (bias_dma_rdata),
        .m_axi_rvalid  (bias_dma_rvalid),
        .m_axi_rlast   (bias_dma_rlast),
        .m_axi_rready  (bias_dma_rready),
        // bias_rom write
        .bias_ld_en    (bias_ld_en),
        .bias_ld_addr  (bias_ld_addr),
        .bias_ld_data  (bias_ld_data)
    );

    // =========================================================================
    // CNN core
    // =========================================================================
    cnn_core #(
        .DATA_WIDTH      (DATA_WIDTH),
        .ACC_WIDTH       (ACC_WIDTH),
        .PE_ROWS         (PE_ROWS),
        .PE_COLS         (PE_COLS),
        .BUF_DEPTH_IFMAP (262144),
        .BUF_AW_IFMAP    (18),
        .BUF_DEPTH_OFMAP (262144),
        .BUF_AW_OFMAP    (18),
        .WB_DEPTH        (589824),   // largest single conv layer (L10/L12/L14); was 1048576 (sim-only)
        .WB_AW           (20)        // $clog2(589824)=20, unchanged
    ) u_cnn_core (
        .clk            (sys_clk),
        .rst            (sys_rst),
        .in_channels    (sched_in_ch),
        .out_channels   (sched_out_ch),
        .kernel_size    (sched_kernel),
        .stride         (sched_stride),
        .pad_en         (sched_pad),
        .act_mode       (sched_act),
        .pool_en        (sched_pool),
        .img_dim        (sched_dim),
        .layer_type     (sched_type),
        .rq_multiplier  (sched_rq_mult),
        .rq_shift       (sched_rq_shift),
        .rq_zero_point  (sched_rq_zp),
        .compute_start  (sched_compute_start),
        .compute_done   (core_compute_done),
        .wt_ld_en       (wt_ld_en),
        .wt_ld_addr     (wt_ld_addr),
        .wt_ld_data     ($signed(wt_ld_data)),
        .wt_swap        (sched_buf_swap),
        .bias_ld_en     (bias_ld_en),
        .bias_ld_addr   (bias_ld_addr),
        .bias_ld_data   (bias_ld_data),
        .ifmap_ext_wr_en  (ifmap_wr_en),
        .ifmap_ext_wr_addr(ifmap_wr_addr),
        .ifmap_ext_wr_data(ifmap_wr_data),
        .out_data       (core_out_data),
        .out_valid      (core_out_valid),
        .out_last       (core_out_last)
    );

    // =========================================================================
    // Reorder DMA — captures cnn_core output, scatters to ifmap_buffer
    // =========================================================================
    reorder_dma #(
        .DATA_WIDTH (DATA_WIDTH),
        .PE_ROWS    (PE_ROWS),
        .MAX_OUT    (262144),
        .STAGE_AW   (18),
        .IFMAP_AW   (18),
        .MAX_DIM    (`MAX_IMG_DIM),
        .MAX_CH     (`MAX_CHANNELS)
    ) u_reorder_dma (
        .clk           (sys_clk),
        .rst           (sys_rst),
        .out_channels  (sched_out_ch),
        .out_dim       (sched_dim),
        .pool_mode     (sched_pool),
        .last_layer    (is_last_layer),
        // Phase 1: capture stream
        .stream_data   (core_out_data),
        .stream_valid  (core_out_valid),
        .stream_last   (core_out_last),
        // Phase 2: scatter trigger
        .reorder_start (core_compute_done),
        .reorder_done  (reorder_done),
        // ifmap_buffer scatter write
        .ifmap_wr_en   (rdma_ifmap_wr_en),
        .ifmap_wr_addr (rdma_ifmap_wr_addr),
        .ifmap_wr_data (rdma_ifmap_wr_data)
    );

    // =========================================================================
    // Tensor output buffer — receives L18 output directly from cnn_core
    // =========================================================================
    tensor_output_buffer u_tensor_buf (
        .clk          (sys_clk),
        .rst          (sys_rst),
        .wr_en        (core_out_valid & is_last_layer),
        .wr_data      (core_out_data),
        .tensor_clear (ctrl_tensor_clear | ctrl_inference_start),
        .rd_en        (tensor_rd_en),
        .rd_addr      (tensor_rd_addr),
        .rd_data      (tensor_rd_data),
        .rd_valid     (tensor_rd_valid),
        .tensor_ready (tensor_ready)
    );

    // =========================================================================
    // AXI-Lite control registers (PS GP0)
    // =========================================================================
    edgeai_ctrl_regs u_ctrl_regs (
        .clk              (sys_clk),
        .rst              (sys_rst),
        // AXI-Lite from PS GP0
        .s_axi_awaddr     (gp0_awaddr),
        .s_axi_awvalid    (gp0_awvalid),
        .s_axi_awready    (gp0_awready),
        .s_axi_wdata      (gp0_wdata),
        .s_axi_wstrb      (gp0_wstrb),
        .s_axi_wvalid     (gp0_wvalid),
        .s_axi_wready     (gp0_wready),
        .s_axi_bresp      (gp0_bresp),
        .s_axi_bvalid     (gp0_bvalid),
        .s_axi_bready     (gp0_bready),
        .s_axi_araddr     (gp0_araddr),
        .s_axi_arvalid    (gp0_arvalid),
        .s_axi_arready    (gp0_arready),
        .s_axi_rdata      (gp0_rdata),
        .s_axi_rresp      (gp0_rresp),
        .s_axi_rvalid     (gp0_rvalid),
        .s_axi_rready     (gp0_rready),
        // Control pulses
        .inference_start  (ctrl_inference_start),
        .cnn_soft_reset   (ctrl_cnn_soft_reset),
        .tensor_clear     (ctrl_tensor_clear),
        .wt_base_addr     (ctrl_wt_base),
        .bias_base_addr   (ctrl_bias_base),
        .frame_buf_a      (ctrl_frame_a),
        .frame_buf_b      (ctrl_frame_b),
        .tensor_dst       (ctrl_tensor_dst),
        // Status from PL
        .busy             (busy),
        .tensor_ready     (tensor_ready),
        .inference_done   (top_inference_done),
        .cam_frame_error  (cam_frame_error)
    );

endmodule
