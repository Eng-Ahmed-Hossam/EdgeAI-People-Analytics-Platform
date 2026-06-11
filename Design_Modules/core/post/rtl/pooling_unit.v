/*
------------------------------------------------------------------------------
Module Name : pooling_unit
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    2x2 max pooling unit with stride 2.
    
    Functional Role:
    - Implements max pooling over 2x2 windows
    - Uses line buffer to store one row of activated values
    - Outputs one max value per 2x2 window
    - Bypass mode for layers without pooling
    
    Input/Output Contract:
    - data_in: Signed INT8 activated value
    - data_out: Signed INT8 max of 2x2 window
    - row_size: Current feature map width
    - bypass: When HIGH, passes data through without pooling
    - in_valid/out_valid: Pipeline valid handshake
    
    Data Widths:
    - Input/Output: DATA_WIDTH (8-bit)
    
    Timing:
    - Output valid every 2 input rows (stride 2)
    - 1 clock cycle latency from final pixel of 2x2 window
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — pooling

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pooling_unit #(
    parameter DATA_WIDTH  = `DATA_WIDTH,
    parameter MAX_IMG_DIM = `MAX_IMG_DIM
)(
    input  wire                             clk,
    input  wire                             rst,
    
    // Configuration
    input  wire [$clog2(MAX_IMG_DIM+1)-1:0] row_size,    // Feature map width
    input  wire                             bypass,       // Skip pooling
    input  wire                             frame_sync,   // New frame/layer
    
    // Input stream
    input  wire signed [DATA_WIDTH-1:0]     data_in,
    input  wire                             in_valid,
    
    // Output stream
    output reg  signed [DATA_WIDTH-1:0]     data_out,
    output reg                              out_valid
);

    // ========================================================================
    // Line buffer: store previous row for column-wise comparison
    // ========================================================================
    reg signed [DATA_WIDTH-1:0] line_buf [0:MAX_IMG_DIM/2-1];
    reg [$clog2(MAX_IMG_DIM)-1:0] col_cnt;
    reg [$clog2(MAX_IMG_DIM)-1:0] row_cnt;
    
    // Max registers
    reg signed [DATA_WIDTH-1:0] col_max;      // Max of two horizontally adjacent pixels
    reg signed [DATA_WIDTH-1:0] prev_row_max; // Max from previous row
    
    // Position flags
    wire is_even_col;
    wire is_even_row;
    wire is_pool_output;
    
    assign is_even_col = ~col_cnt[0];   // col_cnt[0] == 0 → even column
    assign is_even_row = ~row_cnt[0];   // row_cnt[0] == 0 → even row
    assign is_pool_output = !is_even_col && !is_even_row;  // Odd col, odd row → 2x2 complete
    
    // ========================================================================
    // Signed max function
    // ========================================================================
    function signed [DATA_WIDTH-1:0] smax;
        input signed [DATA_WIDTH-1:0] a;
        input signed [DATA_WIDTH-1:0] b;
    begin
        smax = (a > b) ? a : b;
    end
    endfunction
    
    // ========================================================================
    // Pooling datapath
    // ========================================================================
    always @(posedge clk) begin
        if (rst || frame_sync) begin
            col_cnt      <= 0;
            row_cnt      <= 0;
            col_max      <= {1'b1, {(DATA_WIDTH-1){1'b0}}};  // INT8_MIN
            prev_row_max <= {1'b1, {(DATA_WIDTH-1){1'b0}}};  // INT8_MIN
            data_out     <= 0;
            out_valid    <= 1'b0;
        end else begin
            out_valid <= 1'b0;
            
            if (in_valid) begin
                if (bypass) begin
                    // Bypass mode: pass through directly
                    data_out  <= data_in;
                    out_valid <= 1'b1;
                end else begin
                    // ========================================================
                    // 2x2 max pooling state machine
                    // ========================================================
                    
                    if (is_even_col) begin
                        // Even column: just store for horizontal comparison
                        col_max <= data_in;
                    end else begin
                        // Odd column: compute horizontal max
                        // Max of current pair (even_col, odd_col)
                        if (is_even_row) begin
                            // Even row: store row max in line buffer for vertical comparison
                            line_buf[col_cnt >> 1] <= smax(col_max, data_in);
                        end else begin
                            // Odd row: compute final 2x2 max
                            prev_row_max <= line_buf[col_cnt >> 1];
                            data_out  <= smax(smax(col_max, data_in), line_buf[col_cnt >> 1]);
                            out_valid <= 1'b1;
                        end
                    end
                    
                    // Update position
                    if (col_cnt == row_size - 1) begin
                        col_cnt <= 0;
                        row_cnt <= row_cnt + 1;
                    end else begin
                        col_cnt <= col_cnt + 1;
                    end
                end
            end
        end
    end

endmodule
