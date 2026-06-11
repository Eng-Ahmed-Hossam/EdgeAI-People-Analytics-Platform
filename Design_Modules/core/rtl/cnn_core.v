/*
------------------------------------------------------------------------------
Module Name : cnn_core
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    CNN core wrapper integrating compute, post-processing, and buffers.
    
    Functional Role:
    - Instantiates and interconnects:
      * PE array (compute)
      * Window generator (sliding window)
      * Bias adder, Requantizer, Activation, Pooling (post-conv pipeline)
      * ifmap_buffer, ofmap_buffer, weight_buffer (storage)
    - Provides unified interface to layer_scheduler
    - Manages data flow between all sub-modules
    
    Input/Output Contract:
    - Layer config from scheduler
    - Weight load interface
    - Compute start/done handshake
    - Buffer swap control
    - Final output data interface for tensor output
    
    Data Widths:
    - Per sub-module specifications
    
    Timing:
    - Pipeline latency: window_gen(1) + PE(1) + bias(1) + requant(2) + act(1) + pool(1) = ~7 cycles
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — core integration

Dependencies:
    pe_array.v, window_gen.v, bias_add.v, requantizer.v,
    activation_unit.v, pooling_unit.v, ifmap_buffer.v,
    ofmap_buffer.v, weight_buffer.v, edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module cnn_core #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter ACC_WIDTH  = `ACC_WIDTH,
    parameter PE_ROWS    = `PE_ROWS,
    parameter PE_COLS    = `PE_COLS,
    parameter MAX_K      = `MAX_KERNEL_SIZE,
    parameter MAX_DIM    = `MAX_IMG_DIM,
    parameter MAX_CH     = `MAX_CHANNELS,
    parameter BUF_DEPTH_FMAP = MAX_DIM * MAX_DIM,
    parameter BUF_AW_FMAP    = $clog2(BUF_DEPTH_FMAP),
    parameter WB_DEPTH   = 65536,
    parameter WB_AW      = $clog2(WB_DEPTH)
)(
    input  wire        clk,
    input  wire        rst,
    
    // Layer configuration (from layer_scheduler)
    input  wire [8:0]  in_channels,
    input  wire [8:0]  out_channels,
    input  wire [1:0]  kernel_size,
    input  wire        stride,
    input  wire        pad_en,
    input  wire [1:0]  act_mode,
    input  wire        pool_en,
    input  wire [$clog2(MAX_DIM+1)-1:0] img_dim,
    input  wire        layer_type,
    
    // Requantization parameters
    input  wire [`MULT_WIDTH-1:0]      rq_multiplier,
    input  wire [`SHIFT_WIDTH-1:0]     rq_shift,
    input  wire signed [DATA_WIDTH-1:0] rq_zero_point,
    
    // Compute control
    input  wire        compute_start,
    output reg         compute_done,
    
    // Weight load interface (external → weight_buffer)
    input  wire                         wt_ld_en,
    input  wire [WB_AW-1:0]            wt_ld_addr,
    input  wire signed [DATA_WIDTH-1:0] wt_ld_data,
    input  wire                         wt_swap,
    
    // ifmap write interface (from cnn_input_adapter, first layer only)
    input  wire                         ifmap_ext_wr_en,
    input  wire [BUF_AW_FMAP-1:0]      ifmap_ext_wr_addr,
    input  wire signed [DATA_WIDTH-1:0] ifmap_ext_wr_data,
    
    // Output data (to tensor_output_buffer)
    output wire signed [DATA_WIDTH-1:0] out_data,
    output wire                         out_valid,
    output wire                         out_last
);

    // ========================================================================
    // Internal signals
    // ========================================================================
    
    // ifmap buffer read
    wire                         ifmap_rd_en;
    wire [BUF_AW_FMAP-1:0]      ifmap_rd_addr;
    wire signed [DATA_WIDTH-1:0] ifmap_rd_data;
    wire                         ifmap_rd_valid;
    
    // weight buffer read
    wire                         wt_rd_en;
    wire [WB_AW-1:0]            wt_rd_addr;
    wire signed [DATA_WIDTH-1:0] wt_rd_data;
    wire                         wt_rd_valid;
    
    // ── Bias ROM (simulation: loaded via $readmemh) ─────────────────────
    localparam BIAS_DEPTH = MAX_CH;  // One bias per output channel
    reg signed [`BIAS_WIDTH-1:0] bias_rom [0:BIAS_DEPTH-1];
    reg [8:0] bias_rd_idx;
    wire signed [`BIAS_WIDTH-1:0] bias_val;
    assign bias_val = bias_rom[bias_rd_idx];
    
    // Window generator output
    wire [MAX_K*MAX_K*DATA_WIDTH-1:0] window_data;
    wire                              window_valid;
    
    // Bias index tracks output channel being computed
    // (placed after window_valid declaration to avoid forward reference)
    always @(posedge clk) begin
        if (rst || compute_start)
            bias_rd_idx <= 0;
        else if (window_valid && bias_rd_idx < out_channels - 1)
            bias_rd_idx <= bias_rd_idx + 1;
    end
    
    // Zero-padded window data for PE array input (window is K*K*DW, PE needs PE_COLS*DW)
    localparam WINDOW_BITS = MAX_K * MAX_K * DATA_WIDTH;  // 72
    localparam PE_IN_BITS  = PE_COLS * DATA_WIDTH;        // 128
    wire [PE_IN_BITS-1:0] window_padded;
    generate
        if (PE_IN_BITS > WINDOW_BITS) begin : pad_window
            assign window_padded = {{(PE_IN_BITS - WINDOW_BITS){1'b0}}, window_data};
        end else begin : trunc_window
            assign window_padded = window_data[PE_IN_BITS-1:0];
        end
    endgenerate
    
    // PE array signals
    wire signed [PE_COLS*DATA_WIDTH-1:0]          pe_ifmap;
    wire signed [PE_ROWS*PE_COLS*DATA_WIDTH-1:0]  pe_weights;
    wire signed [PE_ROWS*PE_COLS*ACC_WIDTH-1:0]   pe_acc_out;
    wire                                          pe_en;
    wire                                          pe_clear;
    wire [PE_ROWS-1:0]                            pe_row_en;
    wire [PE_COLS-1:0]                             pe_col_en;
    
    // Post-processing pipeline
    wire signed [ACC_WIDTH-1:0]  bias_result;
    wire                         bias_valid;
    wire signed [DATA_WIDTH-1:0] rq_result;
    wire                         rq_valid;
    wire signed [DATA_WIDTH-1:0] act_result;
    wire                         act_valid;
    wire signed [DATA_WIDTH-1:0] pool_result;
    wire                         pool_valid;
    
    // ofmap buffer write
    wire                          ofmap_wr_en;
    wire [BUF_AW_FMAP-1:0]       ofmap_wr_addr;
    wire signed [DATA_WIDTH-1:0]  ofmap_wr_data;
    
    // ========================================================================
    // Compute FSM (simplified)
    // ========================================================================
    // This FSM orchestrates reading ifmap, feeding window_gen, 
    // selecting PE inputs, and managing the post-processing pipeline.
    // A full implementation would handle tiled computation for large channels.
    
    localparam CS_IDLE     = 3'd0;
    localparam CS_LOAD     = 3'd1;
    localparam CS_COMPUTE  = 3'd2;
    localparam CS_DRAIN    = 3'd3;
    localparam CS_DONE     = 3'd4;
    
    reg [2:0]  cs_state;
    reg [31:0] cs_pixel_cnt;
    reg [31:0] cs_total_pixels;
    reg        cs_pipeline_done;
    reg [31:0] ofmap_wr_idx;
    
    always @(posedge clk) begin
        if (rst) begin
            cs_state       <= CS_IDLE;
            compute_done   <= 1'b0;
            cs_pixel_cnt   <= 0;
            cs_total_pixels<= 0;
            cs_pipeline_done <= 1'b0;
            ofmap_wr_idx   <= 0;
        end else begin
            compute_done <= 1'b0;
            
            case (cs_state)
                CS_IDLE: begin
                    if (compute_start) begin
                        cs_total_pixels <= img_dim * img_dim;
                        cs_pixel_cnt    <= 0;
                        ofmap_wr_idx    <= 0;
                        cs_state        <= CS_COMPUTE;
                    end
                end
                
                CS_COMPUTE: begin
                    // Stream pixels through pipeline
                    if (cs_pixel_cnt >= cs_total_pixels) begin
                        cs_state <= CS_DRAIN;
                    end else begin
                        cs_pixel_cnt <= cs_pixel_cnt + 1;
                    end
                end
                
                CS_DRAIN: begin
                    // Wait for pipeline to flush (conservative: 16 cycles)
                    if (cs_pixel_cnt >= cs_total_pixels + 16) begin
                        cs_state <= CS_DONE;
                    end else begin
                        cs_pixel_cnt <= cs_pixel_cnt + 1;
                    end
                end
                
                CS_DONE: begin
                    compute_done <= 1'b1;
                    cs_state     <= CS_IDLE;
                end
                
                default: cs_state <= CS_IDLE;
            endcase
        end
    end
    
    // ========================================================================
    // Pipeline control signals
    // ========================================================================
    assign pe_en     = (cs_state == CS_COMPUTE);
    assign pe_clear  = (cs_state == CS_IDLE);
    assign pe_row_en = {PE_ROWS{1'b1}};   // All rows active
    assign pe_col_en = {PE_COLS{1'b1}};   // All cols active
    assign ifmap_rd_en = (cs_state == CS_COMPUTE);
    assign wt_rd_en    = (cs_state == CS_COMPUTE);
    
    // Simplified: use first PE accumulator output for post-processing demo
    // Full implementation would iterate over all PE outputs
    wire signed [ACC_WIDTH-1:0] pe_out_selected;
    assign pe_out_selected = pe_acc_out[ACC_WIDTH-1:0];
    
    // ========================================================================
    // ifmap buffer
    // ========================================================================
    // Address generation (simplified: sequential read)
    reg [BUF_AW_FMAP-1:0] ifmap_read_addr;
    always @(posedge clk) begin
        if (rst || compute_start)
            ifmap_read_addr <= 0;
        else if (ifmap_rd_en)
            ifmap_read_addr <= ifmap_read_addr + 1;
    end
    assign ifmap_rd_addr = ifmap_read_addr;
    
    ifmap_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (BUF_DEPTH_FMAP)
    ) u_ifmap_buf (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (ifmap_ext_wr_en),
        .wr_addr (ifmap_ext_wr_addr),
        .wr_data (ifmap_ext_wr_data),
        .rd_en   (ifmap_rd_en),
        .rd_addr (ifmap_rd_addr),
        .rd_data (ifmap_rd_data),
        .rd_valid(ifmap_rd_valid)
    );
    
    // ========================================================================
    // Weight buffer
    // ========================================================================
    reg [WB_AW-1:0] wt_read_addr;
    always @(posedge clk) begin
        if (rst || compute_start)
            wt_read_addr <= 0;
        else if (wt_rd_en)
            wt_read_addr <= wt_read_addr + 1;
    end
    assign wt_rd_addr = wt_read_addr;
    
    weight_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (WB_DEPTH)
    ) u_weight_buf (
        .clk     (clk),
        .rst     (rst),
        .ld_en   (wt_ld_en),
        .ld_addr (wt_ld_addr),
        .ld_data (wt_ld_data),
        .rd_en   (wt_rd_en),
        .rd_addr (wt_rd_addr),
        .rd_data (wt_rd_data),
        .rd_valid(wt_rd_valid),
        .swap    (wt_swap)
    );
    
    // ========================================================================
    // Window generator
    // ========================================================================
    window_gen #(
        .DATA_WIDTH  (DATA_WIDTH),
        .MAX_K       (MAX_K),
        .MAX_IMG_DIM (MAX_DIM)
    ) u_window_gen (
        .clk         (clk),
        .rst         (rst),
        .kernel_size (kernel_size),
        .row_size    (img_dim),
        .pad_en      (pad_en),
        .stride      ({1'b0, stride}),
        .pixel_in    (ifmap_rd_data),
        .pixel_valid (ifmap_rd_valid),
        .frame_sync  (compute_start),
        .window_data (window_data),
        .window_valid(window_valid)
    );
    
    // ========================================================================
    // PE array connection
    // ========================================================================
    // Window data feeds PE columns (spatial positions of the convolution window)
    // Weight buffer provides sequential weights: the compute FSM reads one weight
    // per cycle, broadcasting it to all PE rows for the current input position.
    // Each PE row accumulates for a different output channel.
    assign pe_ifmap   = window_padded;                          // Zero-padded window → PE columns
    assign pe_weights = {PE_ROWS*PE_COLS{wt_rd_data}};        // Broadcast weight to all PEs
    
    pe_array #(
        .PE_ROWS    (PE_ROWS),
        .PE_COLS    (PE_COLS),
        .DATA_WIDTH (DATA_WIDTH),
        .ACC_WIDTH  (ACC_WIDTH)
    ) u_pe_array (
        .clk        (clk),
        .rst        (rst),
        .en         (pe_en & window_valid),
        .acc_clear  (pe_clear),
        .row_en     (pe_row_en),
        .col_en     (pe_col_en),
        .ifmap_data (pe_ifmap),
        .weight_data(pe_weights),
        .acc_out    (pe_acc_out)
    );
    
    // ========================================================================
    // Post-processing pipeline
    // ========================================================================
    
    // Bias (from bias ROM — loaded via $readmemh in simulation)
    bias_add #(
        .ACC_WIDTH  (ACC_WIDTH),
        .BIAS_WIDTH (`BIAS_WIDTH)
    ) u_bias_add (
        .clk      (clk),
        .rst      (rst),
        .acc_in   (pe_out_selected),
        .bias_in  (bias_val),
        .in_valid (window_valid),
        .result   (bias_result),
        .out_valid(bias_valid)
    );
    
    // Requantizer
    requantizer #(
        .ACC_WIDTH   (ACC_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .MULT_WIDTH  (`MULT_WIDTH),
        .SHIFT_WIDTH (`SHIFT_WIDTH)
    ) u_requantizer (
        .clk        (clk),
        .rst        (rst),
        .data_in    (bias_result),
        .in_valid   (bias_valid),
        .multiplier (rq_multiplier),
        .shift      (rq_shift),
        .zero_point (rq_zero_point),
        .data_out   (rq_result),
        .out_valid  (rq_valid)
    );
    
    // Activation
    activation_unit #(
        .DATA_WIDTH (DATA_WIDTH),
        .LEAK_SHIFT (`LEAK_SHIFT)
    ) u_activation (
        .clk      (clk),
        .rst      (rst),
        .data_in  (rq_result),
        .in_valid (rq_valid),
        .mode     (act_mode),
        .data_out (act_result),
        .out_valid(act_valid)
    );
    
    // Pooling
    pooling_unit #(
        .DATA_WIDTH  (DATA_WIDTH),
        .MAX_IMG_DIM (MAX_DIM)
    ) u_pooling (
        .clk        (clk),
        .rst        (rst),
        .row_size   (img_dim),
        .bypass     (~pool_en),
        .frame_sync (compute_start),
        .data_in    (act_result),
        .in_valid   (act_valid),
        .data_out   (pool_result),
        .out_valid  (pool_valid)
    );
    
    // ========================================================================
    // ofmap buffer
    // ========================================================================
    assign ofmap_wr_en   = pool_valid;
    assign ofmap_wr_data = pool_result;
    
    always @(posedge clk) begin
        if (rst || compute_start)
            ofmap_wr_idx <= 0;
        else if (pool_valid)
            ofmap_wr_idx <= ofmap_wr_idx + 1;
    end
    assign ofmap_wr_addr = ofmap_wr_idx[BUF_AW_FMAP-1:0];
    
    ofmap_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (BUF_DEPTH_FMAP)
    ) u_ofmap_buf (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (ofmap_wr_en),
        .wr_addr (ofmap_wr_addr),
        .wr_data (ofmap_wr_data),
        .rd_en   (1'b0),           // Read controlled externally or during swap
        .rd_addr ({BUF_AW_FMAP{1'b0}}),
        .rd_data (),
        .rd_valid()
    );
    
    // ========================================================================
    // Output interface
    // ========================================================================
    assign out_data  = pool_result;
    assign out_valid = pool_valid;
    assign out_last  = (cs_state == CS_DONE);

endmodule
