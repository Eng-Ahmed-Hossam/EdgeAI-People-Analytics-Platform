/*
------------------------------------------------------------------------------
Module Name : layer_scheduler
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team
Description :
    Layer iteration FSM for sequential Tiny-YOLO inference.
    
    Functional Role:
    - Steps through all 19 layers of Tiny-YOLO (256-channel cap variant)
    - Configures each layer's parameters (channels, kernel, stride, pooling)
    - Provides per-layer requantizer parameters (multiplier, shift, zero_point)
    - Controls weight loading, computation, and buffer swapping
    - Manages inter-layer handshaking
    
    Input/Output Contract:
    - start: Begin inference on a new frame
    - done: Inference complete, tensor ready
    - Layer config outputs: kernel_size, stride, pad_en, in_channels,
      out_channels, act_mode, pool_en, img_dim, layer_type
    - Requantizer outputs: rq_multiplier, rq_shift, rq_zero_point
    - Computation control: compute_start, compute_done
    - Buffer control: buf_swap
    
    Timing:
    - One layer at a time, sequential
    - Weight preload overlaps with current computation (double-buffer)
    
    Reset:
    - Synchronous active-HIGH reset
    - Returns to IDLE state

Dataflow    :
    Output-Stationary CNN accelerator pipeline — layer sequencing

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
    v2.0 - Added requantizer ROM, corrected topology to match export
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module layer_scheduler #(
    parameter NUM_LAYERS = `NUM_LAYERS,
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter MAX_CH     = `MAX_CHANNELS,
    parameter MAX_DIM    = `MAX_IMG_DIM
)(
    input  wire       clk,
    input  wire       rst,
    
    // Control
    input  wire       start,          // Begin inference
    output reg        done,           // Inference complete
    output reg        busy,           // Currently processing
    
    // Layer configuration outputs
    output reg [$clog2(NUM_LAYERS)-1:0] current_layer,
    output reg [8:0]                    in_channels,     // Up to 256
    output reg [8:0]                    out_channels,    // Up to 255
    output reg [1:0]                    kernel_size,     // 1 or 3
    output reg                          stride,          // 1 or 2
    output reg                          pad_en,          // Zero padding enable
    output reg [1:0]                    act_mode,        // Activation mode
    output reg                          pool_en,         // Pooling enable
    output reg [7:0]                    img_dim,         // Spatial dimension
    output reg                          layer_type,      // CONV or POOL
    
    // Requantization parameters
    output reg [`MULT_WIDTH-1:0]        rq_multiplier,
    output reg [`SHIFT_WIDTH-1:0]       rq_shift,
    output reg signed [DATA_WIDTH-1:0]  rq_zero_point,
    
    // Weight buffer control
    output reg        wt_load_start,  // Start weight preload
    input  wire       wt_load_done,   // Weight load complete
    
    // Compute control
    output reg        compute_start,  // Start layer computation
    input  wire       compute_done,   // Layer computation complete
    
    // Buffer swap
    output reg        buf_swap        // Swap ifmap/ofmap buffers
);

    // ========================================================================
    // Layer configuration ROM
    // ========================================================================
    // Tiny-YOLO-256 layer configurations (from export_tinyyolo_rtl.py)
    // Topology: 13 conv + 6 pool = 19 layers total
    //
    // L0:  CONV 3→16   3x3  dim=128
    // L1:  POOL         2x2  dim=128→64
    // L2:  CONV 16→32  3x3  dim=64
    // L3:  POOL         2x2  dim=64→32
    // L4:  CONV 32→64  3x3  dim=32
    // L5:  POOL         2x2  dim=32→16
    // L6:  CONV 64→128 3x3  dim=16
    // L7:  POOL         2x2  dim=16→8
    // L8:  CONV 128→256 3x3 dim=8
    // L9:  POOL         2x2  dim=8→4
    // L10: CONV 256→256 3x3 dim=4
    // L11: POOL         2x2  dim=4 stride=1 (spatial preservation)
    // L12: CONV 256→256 3x3 dim=4
    // L13: CONV 256→256 1x1 dim=4
    // L14: CONV 256→256 3x3 dim=4
    // L15: CONV 256→255 1x1 dim=4 (detection head #1, LINEAR)
    // L16: CONV 255→128 1x1 dim=4
    // L17: CONV 128→256 3x3 dim=4
    // L18: CONV 256→255 1x1 dim=4 (detection head #2, LINEAR)
    
    // Layer config registers (loaded from ROM)
    reg [8:0]  rom_in_ch   [0:NUM_LAYERS-1];
    reg [8:0]  rom_out_ch  [0:NUM_LAYERS-1];
    reg [1:0]  rom_kernel  [0:NUM_LAYERS-1];
    reg        rom_stride  [0:NUM_LAYERS-1];
    reg        rom_pad     [0:NUM_LAYERS-1];
    reg [1:0]  rom_act     [0:NUM_LAYERS-1];
    reg        rom_pool    [0:NUM_LAYERS-1];
    reg [7:0]  rom_dim     [0:NUM_LAYERS-1];
    reg        rom_type    [0:NUM_LAYERS-1];
    
    // Requantizer parameter ROM (auto-generated from export script)
    reg [`MULT_WIDTH-1:0]       rom_rq_mult  [0:NUM_LAYERS-1];
    reg [`SHIFT_WIDTH-1:0]      rom_rq_shift [0:NUM_LAYERS-1];
    reg signed [DATA_WIDTH-1:0] rom_rq_zp    [0:NUM_LAYERS-1];
    
    // Initialize configuration ROM
    initial begin
        // Layer 0: Conv 3→16, 3x3, stride 1, pad 1, LeakyReLU
        rom_in_ch[0]  = 9'd3;     rom_out_ch[0]  = 9'd16;
        rom_kernel[0] = 2'd3;     rom_stride[0]  = 1'b0;  // stride 1
        rom_pad[0]    = 1'b1;     rom_act[0]     = `ACT_LEAKY_RELU;
        rom_pool[0]   = 1'b0;     rom_dim[0]     = 8'd128;
        rom_type[0]   = `LAYER_TYPE_CONV;
        
        // Layer 1: MaxPool 2x2 stride 2
        rom_in_ch[1]  = 9'd16;    rom_out_ch[1]  = 9'd16;
        rom_kernel[1] = 2'd2;     rom_stride[1]  = 1'b1;  // stride 2
        rom_pad[1]    = 1'b0;     rom_act[1]     = `ACT_NONE;
        rom_pool[1]   = 1'b1;     rom_dim[1]     = 8'd128;
        rom_type[1]   = `LAYER_TYPE_POOL;
        
        // Layer 2: Conv 16→32, 3x3
        rom_in_ch[2]  = 9'd16;    rom_out_ch[2]  = 9'd32;
        rom_kernel[2] = 2'd3;     rom_stride[2]  = 1'b0;
        rom_pad[2]    = 1'b1;     rom_act[2]     = `ACT_LEAKY_RELU;
        rom_pool[2]   = 1'b0;     rom_dim[2]     = 8'd64;
        rom_type[2]   = `LAYER_TYPE_CONV;
        
        // Layer 3: MaxPool 2x2
        rom_in_ch[3]  = 9'd32;    rom_out_ch[3]  = 9'd32;
        rom_kernel[3] = 2'd2;     rom_stride[3]  = 1'b1;
        rom_pad[3]    = 1'b0;     rom_act[3]     = `ACT_NONE;
        rom_pool[3]   = 1'b1;     rom_dim[3]     = 8'd64;
        rom_type[3]   = `LAYER_TYPE_POOL;
        
        // Layer 4: Conv 32→64, 3x3
        rom_in_ch[4]  = 9'd32;    rom_out_ch[4]  = 9'd64;
        rom_kernel[4] = 2'd3;     rom_stride[4]  = 1'b0;
        rom_pad[4]    = 1'b1;     rom_act[4]     = `ACT_LEAKY_RELU;
        rom_pool[4]   = 1'b0;     rom_dim[4]     = 8'd32;
        rom_type[4]   = `LAYER_TYPE_CONV;
        
        // Layer 5: MaxPool 2x2
        rom_in_ch[5]  = 9'd64;    rom_out_ch[5]  = 9'd64;
        rom_kernel[5] = 2'd2;     rom_stride[5]  = 1'b1;
        rom_pad[5]    = 1'b0;     rom_act[5]     = `ACT_NONE;
        rom_pool[5]   = 1'b1;     rom_dim[5]     = 8'd32;
        rom_type[5]   = `LAYER_TYPE_POOL;
        
        // Layer 6: Conv 64→128, 3x3
        rom_in_ch[6]  = 9'd64;    rom_out_ch[6]  = 9'd128;
        rom_kernel[6] = 2'd3;     rom_stride[6]  = 1'b0;
        rom_pad[6]    = 1'b1;     rom_act[6]     = `ACT_LEAKY_RELU;
        rom_pool[6]   = 1'b0;     rom_dim[6]     = 8'd16;
        rom_type[6]   = `LAYER_TYPE_CONV;
        
        // Layer 7: MaxPool 2x2
        rom_in_ch[7]  = 9'd128;   rom_out_ch[7]  = 9'd128;
        rom_kernel[7] = 2'd2;     rom_stride[7]  = 1'b1;
        rom_pad[7]    = 1'b0;     rom_act[7]     = `ACT_NONE;
        rom_pool[7]   = 1'b1;     rom_dim[7]     = 8'd16;
        rom_type[7]   = `LAYER_TYPE_POOL;
        
        // Layer 8: Conv 128→256, 3x3
        rom_in_ch[8]  = 9'd128;   rom_out_ch[8]  = 9'd256;
        rom_kernel[8] = 2'd3;     rom_stride[8]  = 1'b0;
        rom_pad[8]    = 1'b1;     rom_act[8]     = `ACT_LEAKY_RELU;
        rom_pool[8]   = 1'b0;     rom_dim[8]     = 8'd8;
        rom_type[8]   = `LAYER_TYPE_CONV;
        
        // Layer 9: MaxPool 2x2
        rom_in_ch[9]  = 9'd256;   rom_out_ch[9]  = 9'd256;
        rom_kernel[9] = 2'd2;     rom_stride[9]  = 1'b1;
        rom_pad[9]    = 1'b0;     rom_act[9]     = `ACT_NONE;
        rom_pool[9]   = 1'b1;     rom_dim[9]     = 8'd8;
        rom_type[9]   = `LAYER_TYPE_POOL;
        
        // Layer 10: Conv 256→256, 3x3
        rom_in_ch[10] = 9'd256;   rom_out_ch[10] = 9'd256;
        rom_kernel[10]= 2'd3;     rom_stride[10] = 1'b0;
        rom_pad[10]   = 1'b1;     rom_act[10]    = `ACT_LEAKY_RELU;
        rom_pool[10]  = 1'b0;     rom_dim[10]    = 8'd4;
        rom_type[10]  = `LAYER_TYPE_CONV;
        
        // Layer 11: MaxPool 2x2 stride=1 (spatial preservation)
        rom_in_ch[11] = 9'd256;   rom_out_ch[11] = 9'd256;
        rom_kernel[11]= 2'd2;     rom_stride[11] = 1'b0;  // stride 1
        rom_pad[11]   = 1'b1;     rom_act[11]    = `ACT_NONE;
        rom_pool[11]  = 1'b1;     rom_dim[11]    = 8'd4;
        rom_type[11]  = `LAYER_TYPE_POOL;
        
        // Layer 12: Conv 256→256, 3x3
        rom_in_ch[12] = 9'd256;   rom_out_ch[12] = 9'd256;
        rom_kernel[12]= 2'd3;     rom_stride[12] = 1'b0;
        rom_pad[12]   = 1'b1;     rom_act[12]    = `ACT_LEAKY_RELU;
        rom_pool[12]  = 1'b0;     rom_dim[12]    = 8'd4;
        rom_type[12]  = `LAYER_TYPE_CONV;
        
        // Layer 13: Conv 256→256, 1x1 (channel reduction/expansion)
        rom_in_ch[13] = 9'd256;   rom_out_ch[13] = 9'd256;
        rom_kernel[13]= 2'd1;     rom_stride[13] = 1'b0;
        rom_pad[13]   = 1'b0;     rom_act[13]    = `ACT_LEAKY_RELU;
        rom_pool[13]  = 1'b0;     rom_dim[13]    = 8'd4;
        rom_type[13]  = `LAYER_TYPE_CONV;
        
        // Layer 14: Conv 256→256, 3x3
        rom_in_ch[14] = 9'd256;   rom_out_ch[14] = 9'd256;
        rom_kernel[14]= 2'd3;     rom_stride[14] = 1'b0;
        rom_pad[14]   = 1'b1;     rom_act[14]    = `ACT_LEAKY_RELU;
        rom_pool[14]  = 1'b0;     rom_dim[14]    = 8'd4;
        rom_type[14]  = `LAYER_TYPE_CONV;
        
        // Layer 15: Conv 256→255, 1x1 — Detection Head #1 (LINEAR, no activation)
        rom_in_ch[15] = 9'd256;   rom_out_ch[15] = 9'd255;
        rom_kernel[15]= 2'd1;     rom_stride[15] = 1'b0;
        rom_pad[15]   = 1'b0;     rom_act[15]    = `ACT_NONE;
        rom_pool[15]  = 1'b0;     rom_dim[15]    = 8'd4;
        rom_type[15]  = `LAYER_TYPE_CONV;
        
        // Layer 16: Conv 255→128, 1x1
        rom_in_ch[16] = 9'd255;   rom_out_ch[16] = 9'd128;
        rom_kernel[16]= 2'd1;     rom_stride[16] = 1'b0;
        rom_pad[16]   = 1'b0;     rom_act[16]    = `ACT_LEAKY_RELU;
        rom_pool[16]  = 1'b0;     rom_dim[16]    = 8'd4;
        rom_type[16]  = `LAYER_TYPE_CONV;
        
        // Layer 17: Conv 128→256, 3x3
        rom_in_ch[17] = 9'd128;   rom_out_ch[17] = 9'd256;
        rom_kernel[17]= 2'd3;     rom_stride[17] = 1'b0;
        rom_pad[17]   = 1'b1;     rom_act[17]    = `ACT_LEAKY_RELU;
        rom_pool[17]  = 1'b0;     rom_dim[17]    = 8'd4;
        rom_type[17]  = `LAYER_TYPE_CONV;
        
        // Layer 18: Conv 256→255, 1x1 — Detection Head #2 (LINEAR)
        rom_in_ch[18] = 9'd256;   rom_out_ch[18] = 9'd255;
        rom_kernel[18]= 2'd1;     rom_stride[18] = 1'b0;
        rom_pad[18]   = 1'b0;     rom_act[18]    = `ACT_NONE;
        rom_pool[18]  = 1'b0;     rom_dim[18]    = 8'd4;
        rom_type[18]  = `LAYER_TYPE_CONV;
        
        // ── Requantizer parameters (from export_tinyyolo_rtl.py) ──
        rom_rq_mult[0]  = 16'd40370;  rom_rq_shift[0]  = 6'd23; rom_rq_zp[0]  = 8'sd0;
        rom_rq_mult[1]  = 16'd0;      rom_rq_shift[1]  = 6'd0;  rom_rq_zp[1]  = 8'sd0;
        rom_rq_mult[2]  = 16'd50412;  rom_rq_shift[2]  = 6'd24; rom_rq_zp[2]  = 8'sd0;
        rom_rq_mult[3]  = 16'd0;      rom_rq_shift[3]  = 6'd0;  rom_rq_zp[3]  = 8'sd0;
        rom_rq_mult[4]  = 16'd36616;  rom_rq_shift[4]  = 6'd24; rom_rq_zp[4]  = 8'sd0;
        rom_rq_mult[5]  = 16'd0;      rom_rq_shift[5]  = 6'd0;  rom_rq_zp[5]  = 8'sd0;
        rom_rq_mult[6]  = 16'd51468;  rom_rq_shift[6]  = 6'd25; rom_rq_zp[6]  = 8'sd0;
        rom_rq_mult[7]  = 16'd0;      rom_rq_shift[7]  = 6'd0;  rom_rq_zp[7]  = 8'sd0;
        rom_rq_mult[8]  = 16'd39603;  rom_rq_shift[8]  = 6'd25; rom_rq_zp[8]  = 8'sd0;
        rom_rq_mult[9]  = 16'd0;      rom_rq_shift[9]  = 6'd0;  rom_rq_zp[9]  = 8'sd0;
        rom_rq_mult[10] = 16'd54718;  rom_rq_shift[10] = 6'd26; rom_rq_zp[10] = 8'sd0;
        rom_rq_mult[11] = 16'd0;      rom_rq_shift[11] = 6'd0;  rom_rq_zp[11] = 8'sd0;
        rom_rq_mult[12] = 16'd56779;  rom_rq_shift[12] = 6'd26; rom_rq_zp[12] = 8'sd0;
        rom_rq_mult[13] = 16'd51601;  rom_rq_shift[13] = 6'd26; rom_rq_zp[13] = 8'sd0;
        rom_rq_mult[14] = 16'd62723;  rom_rq_shift[14] = 6'd26; rom_rq_zp[14] = 8'sd0;
        rom_rq_mult[15] = 16'd41974;  rom_rq_shift[15] = 6'd25; rom_rq_zp[15] = 8'sd0;
        rom_rq_mult[16] = 16'd41738;  rom_rq_shift[16] = 6'd25; rom_rq_zp[16] = 8'sd0;
        rom_rq_mult[17] = 16'd22027;  rom_rq_shift[17] = 6'd31; rom_rq_zp[17] = 8'sd0;
        rom_rq_mult[18] = 16'd62657;  rom_rq_shift[18] = 6'd25; rom_rq_zp[18] = 8'sd0;
    end
    
    // ========================================================================
    // State machine
    // ========================================================================
    reg [3:0] state;
    
    always @(posedge clk) begin
        if (rst) begin
            state         <= `SCHED_IDLE;
            done          <= 1'b0;
            busy          <= 1'b0;
            current_layer <= 0;
            compute_start <= 1'b0;
            wt_load_start <= 1'b0;
            buf_swap      <= 1'b0;
            
            // Clear config outputs
            in_channels   <= 0;
            out_channels  <= 0;
            kernel_size   <= 0;
            stride        <= 0;
            pad_en        <= 0;
            act_mode      <= 0;
            pool_en       <= 0;
            img_dim       <= 0;
            layer_type    <= 0;
            rq_multiplier <= 0;
            rq_shift      <= 0;
            rq_zero_point <= 0;
        end else begin
            // Default pulse signals
            done          <= 1'b0;
            compute_start <= 1'b0;
            wt_load_start <= 1'b0;
            buf_swap      <= 1'b0;
            
            case (state)
                `SCHED_IDLE: begin
                    if (start) begin
                        busy          <= 1'b1;
                        current_layer <= 0;
                        state         <= `SCHED_LOAD_CONFIG;
                    end
                end
                
                `SCHED_LOAD_CONFIG: begin
                    // Load current layer configuration from ROM
                    in_channels   <= rom_in_ch[current_layer];
                    out_channels  <= rom_out_ch[current_layer];
                    kernel_size   <= rom_kernel[current_layer];
                    stride        <= rom_stride[current_layer];
                    pad_en        <= rom_pad[current_layer];
                    act_mode      <= rom_act[current_layer];
                    pool_en       <= rom_pool[current_layer];
                    img_dim       <= rom_dim[current_layer];
                    layer_type    <= rom_type[current_layer];
                    
                    // Load requantizer parameters for this layer
                    rq_multiplier <= rom_rq_mult[current_layer];
                    rq_shift      <= rom_rq_shift[current_layer];
                    rq_zero_point <= rom_rq_zp[current_layer];
                    
                    if (rom_type[current_layer] == `LAYER_TYPE_POOL) begin
                        // Pool layer: skip weight load, go straight to compute
                        state <= `SCHED_COMPUTE;
                        compute_start <= 1'b1;
                    end else begin
                        // Conv layer: load weights first
                        state <= `SCHED_LOAD_WEIGHTS;
                        wt_load_start <= 1'b1;
                    end
                end
                
                `SCHED_LOAD_WEIGHTS: begin
                    if (wt_load_done) begin
                        state <= `SCHED_COMPUTE;
                        compute_start <= 1'b1;
                    end
                end
                
                `SCHED_COMPUTE: begin
                    if (compute_done) begin
                        state <= `SCHED_SWAP;
                    end
                end
                
                `SCHED_SWAP: begin
                    // Swap ifmap/ofmap buffers
                    buf_swap <= 1'b1;
                    state    <= `SCHED_NEXT;
                end
                
                `SCHED_NEXT: begin
                    if (current_layer == NUM_LAYERS - 1) begin
                        // All layers complete
                        state <= `SCHED_DONE;
                    end else begin
                        current_layer <= current_layer + 1;
                        state         <= `SCHED_LOAD_CONFIG;
                    end
                end
                
                `SCHED_DONE: begin
                    done <= 1'b1;
                    busy <= 1'b0;
                    state <= `SCHED_IDLE;
                end
                
                default: state <= `SCHED_IDLE;
            endcase
        end
    end

endmodule
