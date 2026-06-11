/*
------------------------------------------------------------------------------
Module Name : edgeai_defs.vh
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Global parameter definitions for the EdgeAI PL system.
    Contains:
    - Image/frame dimensions
    - Data width definitions for INT8 quantized CNN
    - PE array dimensions
    - Kernel and channel limits
    - YOLO detection output geometry
    - Memory map and address parameters
    - FIFO depths and burst lengths
    - Camera interface parameters

    All modules include this file for consistent parameterization.

Dataflow    :
    Output-Stationary CNN accelerator pipeline

Dependencies:
    None — this is the root parameter file

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`ifndef EDGEAI_DEFS_VH
`define EDGEAI_DEFS_VH

// ============================================================================
// Image / Frame Dimensions
// ============================================================================
`define IMG_WIDTH          128
`define IMG_HEIGHT         128
`define PIXEL_RGB565_WIDTH 16
`define PIXEL_CHANNELS     3

// Camera native resolution (OV7670)
`define CAM_WIDTH          640
`define CAM_HEIGHT         480

// ============================================================================
// Data Width Definitions (INT8 Quantized CNN)
// ============================================================================
`define DATA_WIDTH         8       // INT8 activations and weights
`define ACC_WIDTH          32      // INT32 accumulator output
`define MAC_WIDTH          40      // Internal MAC accumulator (overflow safe)
`define BIAS_WIDTH         32      // INT32 bias
`define MULT_WIDTH         16      // Requantizer multiplier width
`define SHIFT_WIDTH        6       // Requantizer shift bits (max shift 63)

// ============================================================================
// PE Array Dimensions (Output-Stationary, Eyeriss-style)
// ============================================================================
`define PE_ROWS            16      // Number of PE rows (output channels)
`define PE_COLS            16      // Number of PE columns (input channels)

// ============================================================================
// Convolution Parameters
// ============================================================================
`define MAX_KERNEL_SIZE    3       // Maximum kernel dimension (3x3)
`define MAX_CHANNELS       256     // Maximum channel count in network
`define MAX_IMG_DIM        128     // Maximum spatial dimension
`define MAX_STRIDE         2       // Maximum convolution stride

// ============================================================================
// Tiny-YOLO Network Parameters
// ============================================================================
`define NUM_LAYERS         19      // Total layers (conv + pool)
`define NUM_CONV_LAYERS    10      // Number of convolution layers
`define NUM_POOL_LAYERS    9       // Number of pooling layers

// Layer type encoding
`define LAYER_TYPE_CONV    1'b0
`define LAYER_TYPE_POOL    1'b1

// ============================================================================
// YOLO Detection Output Geometry
// ============================================================================
`define GRID_SIZE          4       // Output grid dimension (128 / 2^5 = 4)
`define NUM_ANCHORS        3       // Anchors per grid cell
`define NUM_CLASSES        80      // COCO class count
`define BOX_ATTRS          5       // x, y, w, h, objectness
`define DETECT_CHANNELS    255     // NUM_ANCHORS * (BOX_ATTRS + NUM_CLASSES)

// Tensor output size in bytes
`define TENSOR_SIZE        ((`GRID_SIZE) * (`GRID_SIZE) * (`DETECT_CHANNELS))

// ============================================================================
// Memory Map (SDRAM Address Space)
// ============================================================================
`define SDRAM_ADDR_WIDTH   24      // 16 MB address space
`define SDRAM_DATA_WIDTH   16      // 16-bit SDRAM data bus

// Frame buffer addresses (ping-pong)
`define FRAME_BUF_A_BASE   24'h000000
`define FRAME_BUF_B_BASE   24'h008000  // 128*128*2 = 32KB per frame
`define FRAME_SIZE_BYTES   (`IMG_WIDTH * `IMG_HEIGHT * 2)  // RGB565

// Weight storage base
`define WEIGHT_BASE        24'h010000

// ============================================================================
// FIFO and Burst Parameters
// ============================================================================
`define PIXEL_FIFO_DEPTH   512     // CDC FIFO depth (pixels)
`define PIXEL_FIFO_ADDR_W  9       // log2(512)

`define SDRAM_BURST_LEN    8       // SDRAM burst length
`define SDRAM_BURST_BITS   3       // log2(8)

// ============================================================================
// Camera Interface Parameters
// ============================================================================
`define SCCB_CLK_DIV       250     // System clock / SCCB clock (100MHz/400KHz)
`define XCLK_DIV           4       // System clock / XCLK (100MHz/25MHz)
`define OV7670_ADDR        8'h42   // OV7670 SCCB write address

// ============================================================================
// Activation Function Modes
// ============================================================================
`define ACT_RELU           2'b00
`define ACT_LEAKY_RELU     2'b01
`define ACT_NONE           2'b10

// LeakyReLU approximation: negative slope = 1/2^LEAK_SHIFT
`define LEAK_SHIFT         3       // ~0.125 slope for negatives

// ============================================================================
// Layer Scheduler States
// ============================================================================
`define SCHED_IDLE         4'd0
`define SCHED_LOAD_CONFIG  4'd1
`define SCHED_LOAD_WEIGHTS 4'd2
`define SCHED_FILL_IFMAP   4'd3
`define SCHED_COMPUTE      4'd4
`define SCHED_POST_PROC    4'd5
`define SCHED_POOL         4'd6
`define SCHED_WRITE_OFMAP  4'd7
`define SCHED_SWAP         4'd8
`define SCHED_NEXT         4'd9
`define SCHED_DONE         4'd10

`endif // EDGEAI_DEFS_VH
