/*
------------------------------------------------------------------------------
Module Name : window_gen
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Sliding window generator using line buffers for convolution.
    
    Functional Role:
    - Implements KxK sliding window over 2D feature map
    - Uses line-buffer (shift-register chain) architecture
    - Supports parameterized kernel size, stride, and padding
    - Outputs flattened window data when valid window is available
    
    Input/Output Contract:
    - pixel_in: Single pixel input (INT8)
    - pixel_valid: Input valid strobe
    - window_data: Flattened KxK window output
    - window_valid: Output valid strobe
    - row_size: Width of current feature map
    
    Data Widths:
    - Input pixel: DATA_WIDTH (8-bit)
    - Window output: K*K*DATA_WIDTH bits
    
    Timing:
    - Throughput: one window per valid input pixel (after fill)
    - Initial latency: (K-1) * row_size + (K-1) cycles
    - Stride > 1: window_valid gated by stride counter
    
    Reset:
    - Synchronous active-HIGH reset
    - Clears all line buffers and counters

Dataflow    :
    Output-Stationary CNN accelerator pipeline — window extraction

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module window_gen #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter MAX_K        = `MAX_KERNEL_SIZE,
    parameter MAX_IMG_DIM  = `MAX_IMG_DIM
)(
    input  wire                              clk,
    input  wire                              rst,
    
    // Configuration
    input  wire [$clog2(MAX_K+1)-1:0]        kernel_size,  // 1 or 3
    input  wire [$clog2(MAX_IMG_DIM+1)-1:0]  row_size,     // Feature map width
    input  wire                              pad_en,       // Enable zero padding
    input  wire [$clog2(MAX_K)-1:0]          stride,       // Stride (1 or 2)
    
    // Input pixel stream
    input  wire signed [DATA_WIDTH-1:0]      pixel_in,
    input  wire                              pixel_valid,
    input  wire                              frame_sync,   // Start of new frame
    
    // Output window
    output reg  [MAX_K*MAX_K*DATA_WIDTH-1:0] window_data,
    output reg                               window_valid
);

    // ========================================================================
    // Line buffers: store (K-1) rows for sliding window
    // ========================================================================
    // For 3x3: need 2 line buffers, each MAX_IMG_DIM deep
    
    localparam LINE_BUF_DEPTH = MAX_IMG_DIM;
    localparam NUM_LINE_BUFS  = MAX_K - 1;    // 2 for 3x3
    
    reg signed [DATA_WIDTH-1:0] line_buf [0:NUM_LINE_BUFS-1][0:LINE_BUF_DEPTH-1];
    reg [$clog2(LINE_BUF_DEPTH)-1:0] lb_wr_ptr;
    
    // Shift register taps: 3x3 window
    reg signed [DATA_WIDTH-1:0] tap [0:MAX_K-1][0:MAX_K-1];  // tap[row][col]
    
    // Line buffer outputs
    wire signed [DATA_WIDTH-1:0] lb_out [0:NUM_LINE_BUFS-1];
    
    // ========================================================================
    // Position tracking
    // ========================================================================
    reg [$clog2(MAX_IMG_DIM)-1:0] col_cnt;
    reg [$clog2(MAX_IMG_DIM)-1:0] row_cnt;
    reg valid_region;
    
    // Stride counters
    reg [$clog2(MAX_K)-1:0] stride_col_cnt;
    reg [$clog2(MAX_K)-1:0] stride_row_cnt;
    
    // Fill counter: need K-1 rows + K-1 cols before first valid window
    wire window_row_valid;
    wire window_col_valid;
    wire stride_match;
    
    assign window_row_valid = (row_cnt >= kernel_size - 1) || pad_en;
    assign window_col_valid = (col_cnt >= kernel_size - 1) || pad_en;
    assign stride_match = (stride_col_cnt == 0) && (stride_row_cnt == 0);
    
    // ========================================================================
    // Line buffer read/write
    // ========================================================================
    integer lb_idx;
    
    // Read from line buffers
    generate
        genvar g;
        for (g = 0; g < NUM_LINE_BUFS; g = g + 1) begin : lb_read
            assign lb_out[g] = line_buf[g][lb_wr_ptr];
        end
    endgenerate
    
    // ========================================================================
    // Main datapath
    // ========================================================================
    always @(posedge clk) begin
        if (rst || frame_sync) begin
            col_cnt        <= 0;
            row_cnt        <= 0;
            lb_wr_ptr      <= 0;
            window_valid   <= 1'b0;
            window_data    <= 0;
            stride_col_cnt <= 0;
            stride_row_cnt <= 0;
            valid_region   <= 1'b0;
            
            // Clear taps
            begin : clear_taps
                integer ri, ci;
                for (ri = 0; ri < MAX_K; ri = ri + 1)
                    for (ci = 0; ci < MAX_K; ci = ci + 1)
                        tap[ri][ci] <= 0;
            end
        end else begin
            window_valid <= 1'b0;
            
            if (pixel_valid) begin
                // ============================================================
                // Shift register chain for 3x3 window
                // Row 0 (oldest): from line_buf[1]
                // Row 1 (middle): from line_buf[0]
                // Row 2 (newest): from current input
                // ============================================================
                
                // Shift columns within each row
                begin : shift_cols
                    integer ki;
                    for (ki = MAX_K - 1; ki > 0; ki = ki - 1) begin
                        tap[0][ki] <= tap[0][ki-1];
                        tap[1][ki] <= tap[1][ki-1];
                        tap[2][ki] <= tap[2][ki-1];
                    end
                end
                
                // Feed new values into column 0
                if (NUM_LINE_BUFS >= 2) begin
                    tap[0][0] <= lb_out[1];         // Oldest row from line_buf[1]
                end
                if (NUM_LINE_BUFS >= 1) begin
                    tap[1][0] <= lb_out[0];         // Middle row from line_buf[0]
                end
                tap[2][0] <= pixel_in;              // Newest row from input
                
                // Write to line buffers (shift chain)
                // Line buf 0 gets current input
                line_buf[0][lb_wr_ptr] <= pixel_in;
                // Line buf 1 gets output of line buf 0
                if (NUM_LINE_BUFS >= 2) begin
                    line_buf[1][lb_wr_ptr] <= lb_out[0];
                end
                
                // Update write pointer
                if (col_cnt == row_size - 1) begin
                    lb_wr_ptr <= 0;
                end else begin
                    lb_wr_ptr <= lb_wr_ptr + 1;
                end
                
                // ============================================================
                // Position tracking
                // ============================================================
                if (col_cnt == row_size - 1) begin
                    col_cnt        <= 0;
                    stride_col_cnt <= 0;
                    
                    if (row_cnt == row_size - 1) begin
                        // Assuming square feature map for simplicity
                        row_cnt <= 0;
                    end else begin
                        row_cnt <= row_cnt + 1;
                    end
                    
                    // Row stride tracking
                    if (stride_row_cnt == stride - 1)
                        stride_row_cnt <= 0;
                    else
                        stride_row_cnt <= stride_row_cnt + 1;
                end else begin
                    col_cnt <= col_cnt + 1;
                    
                    // Column stride tracking
                    if (stride_col_cnt == stride - 1)
                        stride_col_cnt <= 0;
                    else
                        stride_col_cnt <= stride_col_cnt + 1;
                end
                
                // ============================================================
                // Output valid window
                // ============================================================
                valid_region <= window_row_valid && window_col_valid;
                
                if (window_row_valid && window_col_valid && stride_match) begin
                    window_valid <= 1'b1;
                    
                    // Pack window data: row-major order
                    // tap[0][0..2] = top row, tap[1][0..2] = mid, tap[2][0..2] = bottom
                    begin : pack_window
                        integer wr, wc;
                        for (wr = 0; wr < MAX_K; wr = wr + 1)
                            for (wc = 0; wc < MAX_K; wc = wc + 1)
                                window_data[(wr*MAX_K+wc)*DATA_WIDTH +: DATA_WIDTH] <= tap[wr][wc];
                    end
                end
            end
        end
    end

endmodule
