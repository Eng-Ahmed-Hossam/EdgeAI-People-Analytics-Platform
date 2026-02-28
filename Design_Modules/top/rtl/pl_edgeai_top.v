/*
------------------------------------------------------------------------------
Module Name : pl_edgeai_top
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Top-level PL module for EdgeAI Tiny-YOLO accelerator.
    
    Functional Role:
    - System-level integration of all sub-systems:
      * OV7670 camera interface (camera_if, sccb_ctrl, xclk_gen)
      * Frame buffering (pixel_fifo, frame_writer, frame_buffer_mgr, sdram_ctrl)
      * CNN accelerator (cnn_core with compute, post-proc, buffers)
      * Layer scheduler (sequential layer control)
      * Tensor output buffer (final detection tensor)
    - Manages end-to-end pipeline: capture → buffer → inference → output
    - Provides PS-accessible tensor read interface
    
    Input/Output Contract:
    - System clock & reset
    - OV7670 camera DVP signals
    - SCCB configuration interface
    - SDRAM physical interface
    - Tensor read port for PS
    - Status/control signals
    
    Timing:
    - System clock: 100 MHz
    - Camera PCLK: ~25 MHz
    - Frame-synchronous inference pipeline
    
    Reset:
    - Synchronous active-HIGH reset
    - Full system reset cascade

Dataflow    :
    Camera → FIFO → SDRAM → CNN Input Adapter → CNN Core → Tensor Output

Dependencies:
    All sub-modules, edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pl_edgeai_top #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter ACC_WIDTH  = `ACC_WIDTH,
    parameter PE_ROWS    = `PE_ROWS,
    parameter PE_COLS    = `PE_COLS,
    parameter SDRAM_AW   = `SDRAM_ADDR_WIDTH,
    parameter SDRAM_DW   = `SDRAM_DATA_WIDTH
)(
    // System
    input  wire        sys_clk,
    input  wire        sys_rst,
    
    // OV7670 Camera DVP interface
    input  wire        cam_pclk,
    input  wire        cam_vsync,
    input  wire        cam_href,
    input  wire [7:0]  cam_d,
    output wire        cam_xclk,
    
    // SCCB interface
    output wire        sccb_scl,
    inout  wire        sccb_sda,
    
    // SDRAM interface
    output wire        sdram_clk,
    output wire        sdram_cke,
    output wire        sdram_cs_n,
    output wire        sdram_ras_n,
    output wire        sdram_cas_n,
    output wire        sdram_we_n,
    output wire [12:0] sdram_addr,
    output wire [1:0]  sdram_ba,
    inout  wire [15:0] sdram_dq,
    output wire [1:0]  sdram_dqm,
    
    // Control
    input  wire        inference_start,   // PS triggers inference
    output wire        inference_done,    // Inference complete
    output wire        tensor_ready,      // Detection tensor readable
    
    // Tensor read interface (PS)
    input  wire        tensor_rd_en,
    input  wire [15:0] tensor_rd_addr,
    output wire signed [DATA_WIDTH-1:0] tensor_rd_data,
    output wire        tensor_rd_valid,
    
    // Status
    output wire        cam_frame_error,
    output wire        busy
);

    // ========================================================================
    // Internal wires
    // ========================================================================
    
    // Camera interface outputs
    wire [15:0] cam_pixel_data;
    wire        cam_pixel_valid;
    wire        cam_frame_start;    // In cam_pclk domain
    wire        cam_frame_end;      // In cam_pclk domain
    
    // CDC synchronizer for cam_frame_start (cam_pclk → sys_clk)
    reg [2:0] frame_start_sync;
    wire      frame_start_sysclk;   // Synchronized rising edge
    
    always @(posedge sys_clk or posedge sys_rst) begin
        if (sys_rst)
            frame_start_sync <= 3'b000;
        else
            frame_start_sync <= {frame_start_sync[1:0], cam_frame_start};
    end
    // Rising edge detect in sys_clk domain
    assign frame_start_sysclk = frame_start_sync[1] & ~frame_start_sync[2];
    
    // Pixel FIFO
    wire        fifo_wr_en;
    wire [15:0] fifo_wr_data;
    wire        fifo_full;
    wire        fifo_almost_full;
    wire [15:0] fifo_rd_data;
    wire        fifo_empty;
    wire        fifo_rd_en;
    
    // Frame writer
    wire        fw_fifo_rd_en;
    wire        fw_write_done;
    wire        fw_sdram_wr_req;
    wire [SDRAM_AW-1:0] fw_sdram_wr_addr;
    wire [SDRAM_DW-1:0] fw_sdram_wr_data;
    wire        fw_sdram_wr_ack;
    
    // Frame buffer manager
    wire [SDRAM_AW-1:0] fbm_wr_base;
    wire [SDRAM_AW-1:0] fbm_rd_base;
    wire        fbm_frame_ready;
    wire        fbm_buf_sel;
    
    // CNN input adapter
    wire        adapter_start;
    wire        adapter_done;
    wire        adapter_sdram_rd_req;
    wire [SDRAM_AW-1:0] adapter_sdram_rd_addr;
    wire [SDRAM_DW-1:0] adapter_sdram_rd_data;
    wire        adapter_sdram_rd_valid;
    wire        adapter_ifmap_wr_en;
    wire [$clog2(`MAX_IMG_DIM*`MAX_IMG_DIM)-1:0] adapter_ifmap_wr_addr;
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
    wire [$clog2(`MAX_IMG_DIM+1)-1:0] sched_dim;
    wire        sched_type;
    wire [`MULT_WIDTH-1:0]  sched_rq_mult;
    wire [`SHIFT_WIDTH-1:0] sched_rq_shift;
    wire signed [DATA_WIDTH-1:0] sched_rq_zp;
    wire        sched_wt_start;
    wire        sched_compute_start;
    wire        sched_buf_swap;
    
    // CNN core
    wire        core_compute_done;
    wire signed [DATA_WIDTH-1:0] core_out_data;
    wire        core_out_valid;
    wire        core_out_last;
    
    // ========================================================================
    // Top-level control FSM
    // ========================================================================
    localparam TOP_IDLE      = 3'd0;
    localparam TOP_WAIT_FRAME= 3'd1;
    localparam TOP_LOAD_FRAME= 3'd2;
    localparam TOP_INFERENCE = 3'd3;
    localparam TOP_DONE      = 3'd4;
    
    reg [2:0] top_state;
    reg       top_inference_done;
    
    assign inference_done = top_inference_done;
    assign busy = (top_state != TOP_IDLE);
    
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            top_state          <= TOP_IDLE;
            top_inference_done <= 1'b0;
        end else begin
            top_inference_done <= 1'b0;
            
            case (top_state)
                TOP_IDLE: begin
                    if (inference_start) begin
                        top_state <= TOP_WAIT_FRAME;
                    end
                end
                
                TOP_WAIT_FRAME: begin
                    if (fbm_frame_ready) begin
                        top_state <= TOP_LOAD_FRAME;
                    end
                end
                
                TOP_LOAD_FRAME: begin
                    if (adapter_done) begin
                        top_state <= TOP_INFERENCE;
                    end
                end
                
                TOP_INFERENCE: begin
                    if (sched_done) begin
                        top_state <= TOP_DONE;
                    end
                end
                
                TOP_DONE: begin
                    top_inference_done <= 1'b1;
                    top_state          <= TOP_IDLE;
                end
                
                default: top_state <= TOP_IDLE;
            endcase
        end
    end
    
    // ========================================================================
    // Control signal generation
    // ========================================================================
    assign adapter_start = (top_state == TOP_WAIT_FRAME) && fbm_frame_ready;
    
    wire sched_start;
    assign sched_start = (top_state == TOP_LOAD_FRAME) && adapter_done;
    
    // ========================================================================
    // Camera subsystem
    // ========================================================================
    
    xclk_gen u_xclk_gen (
        .clk  (sys_clk),
        .rst  (sys_rst),
        .xclk (cam_xclk)
    );
    
    camera_if #(
        .IMG_W      (`IMG_WIDTH),
        .IMG_H      (`IMG_HEIGHT)
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
    
    // ========================================================================
    // Pixel FIFO (CDC: cam_pclk → sys_clk)
    // ========================================================================
    assign fifo_wr_en   = cam_pixel_valid & ~fifo_full;
    assign fifo_wr_data = cam_pixel_data;
    
    pixel_fifo u_pixel_fifo (
        .wr_clk     (cam_pclk),
        .wr_rst     (sys_rst),
        .wr_en      (fifo_wr_en),
        .wr_data    (fifo_wr_data),
        .full       (fifo_full),
        .almost_full(fifo_almost_full),
        .rd_clk     (sys_clk),
        .rd_rst     (sys_rst),
        .rd_en      (fw_fifo_rd_en),
        .rd_data    (fifo_rd_data),
        .empty      (fifo_empty)
    );
    
    // ========================================================================
    // Frame writer
    // ========================================================================
    frame_writer u_frame_writer (
        .clk          (sys_clk),
        .rst          (sys_rst),
        .frame_start  (frame_start_sysclk),  // CDC-synchronized frame start
        .base_addr    (fbm_wr_base),
        .write_done   (fw_write_done),
        .fifo_rd_en   (fw_fifo_rd_en),
        .fifo_data    (fifo_rd_data),
        .fifo_empty   (fifo_empty),
        .sdram_wr_req (fw_sdram_wr_req),
        .sdram_wr_addr(fw_sdram_wr_addr),
        .sdram_wr_data(fw_sdram_wr_data),
        .sdram_wr_ack (fw_sdram_wr_ack)
    );
    
    // ========================================================================
    // Frame buffer manager
    // ========================================================================
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
    
    // ========================================================================
    // SDRAM controller
    // ========================================================================
    sdram_ctrl u_sdram_ctrl (
        .clk          (sys_clk),
        .rst          (sys_rst),
        // Write port (frame writer)
        .wr_req       (fw_sdram_wr_req),
        .wr_addr      (fw_sdram_wr_addr),
        .wr_data      (fw_sdram_wr_data),
        .wr_ack       (fw_sdram_wr_ack),
        // Read port (CNN adapter)
        .rd_req       (adapter_sdram_rd_req),
        .rd_addr      (adapter_sdram_rd_addr),
        .rd_data      (adapter_sdram_rd_data),
        .rd_valid     (adapter_sdram_rd_valid),
        .rd_ack       (),
        // SDRAM physical
        .sdram_clk    (sdram_clk),
        .sdram_cke    (sdram_cke),
        .sdram_cs_n   (sdram_cs_n),
        .sdram_ras_n  (sdram_ras_n),
        .sdram_cas_n  (sdram_cas_n),
        .sdram_we_n   (sdram_we_n),
        .sdram_addr   (sdram_addr),
        .sdram_ba     (sdram_ba),
        .sdram_dq     (sdram_dq),
        .sdram_dqm    (sdram_dqm)
    );
    
    // ========================================================================
    // CNN input adapter
    // ========================================================================
    cnn_input_adapter u_adapter (
        .clk             (sys_clk),
        .rst             (sys_rst),
        .start           (adapter_start),
        .frame_base_addr (fbm_rd_base),
        .done            (adapter_done),
        .sdram_rd_req    (adapter_sdram_rd_req),
        .sdram_rd_addr   (adapter_sdram_rd_addr),
        .sdram_rd_data   (adapter_sdram_rd_data),
        .sdram_rd_valid  (adapter_sdram_rd_valid),
        .ifmap_wr_en     (adapter_ifmap_wr_en),
        .ifmap_wr_addr   (adapter_ifmap_wr_addr),
        .ifmap_wr_data   (adapter_ifmap_wr_data)
    );
    
    // ========================================================================
    // Layer scheduler
    // ========================================================================
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
        .wt_load_done   (1'b1),           // Simplified: instant weight load
        .compute_start  (sched_compute_start),
        .compute_done   (core_compute_done),
        .buf_swap       (sched_buf_swap)
    );
    
    // ========================================================================
    // CNN core
    // ========================================================================
    cnn_core u_cnn_core (
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
        // Weight load (simplified: no external weight load in this version)
        .wt_ld_en       (1'b0),
        .wt_ld_addr     (16'd0),
        .wt_ld_data     ({DATA_WIDTH{1'b0}}),
        .wt_swap        (sched_buf_swap),
        // ifmap from adapter
        .ifmap_ext_wr_en  (adapter_ifmap_wr_en),
        .ifmap_ext_wr_addr(adapter_ifmap_wr_addr),
        .ifmap_ext_wr_data(adapter_ifmap_wr_data),
        // Output
        .out_data       (core_out_data),
        .out_valid      (core_out_valid),
        .out_last       (core_out_last)
    );
    
    // ========================================================================
    // Tensor output buffer
    // ========================================================================
    tensor_output_buffer u_tensor_buf (
        .clk          (sys_clk),
        .rst          (sys_rst),
        .wr_en        (core_out_valid),
        .wr_data      (core_out_data),
        .tensor_clear (inference_start),
        .rd_en        (tensor_rd_en),
        .rd_addr      (tensor_rd_addr),
        .rd_data      (tensor_rd_data),
        .rd_valid     (tensor_rd_valid),
        .tensor_ready (tensor_ready)
    );

endmodule
