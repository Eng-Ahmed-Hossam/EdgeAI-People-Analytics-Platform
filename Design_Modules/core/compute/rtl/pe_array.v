/*
------------------------------------------------------------------------------
Module Name : pe_array
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    NxM Processing Element array with output-stationary dataflow.
    
    Functional Role:
    - Implements Eyeriss-style output-stationary systolic array
    - Rows receive broadcast activation window data
    - Columns receive broadcast weight data
    - Each PE accumulates its own output channel result
    - Supports row/column enable for irregular dimensions
    
    Input/Output Contract:
    - ifmap_row: Broadcast activation to all PEs in a row [PE_COLS × DATA_WIDTH]
    - weight_col: Broadcast weight to all PEs in a column [PE_ROWS × DATA_WIDTH]
    - acc_out: Array of accumulated results [PE_ROWS × PE_COLS × ACC_WIDTH]
    - en: Global enable
    - row_en: Per-row enable mask
    - col_en: Per-column enable mask
    - acc_clear: Clear all accumulators
    
    Data Widths:
    - Activation/Weight: DATA_WIDTH (8-bit)
    - Accumulator: ACC_WIDTH (32-bit)
    
    Timing:
    - One PE_ROWS × PE_COLS array, single-cycle MAC per PE
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary (Eyeriss-style) — each PE accumulates one
    output element across input channels

Dependencies:
    pe.v, edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pe_array #(
    parameter PE_ROWS    = `PE_ROWS,
    parameter PE_COLS    = `PE_COLS,
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter ACC_WIDTH  = `ACC_WIDTH
)(
    input  wire                                       clk,
    input  wire                                       rst,
    input  wire                                       en,          // Global enable
    input  wire                                       acc_clear,   // Clear all accumulators
    input  wire [PE_ROWS-1:0]                         row_en,      // Per-row enable
    input  wire [PE_COLS-1:0]                         col_en,      // Per-column enable
    
    // Broadcast activation data: one value per column (input channel)
    input  wire signed [PE_COLS*DATA_WIDTH-1:0]       ifmap_data,
    
    // Broadcast weight data: one value per row x column
    input  wire signed [PE_ROWS*PE_COLS*DATA_WIDTH-1:0] weight_data,
    
    // Output: accumulated results for each PE
    output wire signed [PE_ROWS*PE_COLS*ACC_WIDTH-1:0] acc_out
);

    // ========================================================================
    // PE array instantiation
    // ========================================================================
    genvar r, c;
    generate
        for (r = 0; r < PE_ROWS; r = r + 1) begin : pe_row
            for (c = 0; c < PE_COLS; c = c + 1) begin : pe_col
                
                // Extract per-PE signals
                wire signed [DATA_WIDTH-1:0] pe_ifmap;
                wire signed [DATA_WIDTH-1:0] pe_weight;
                wire                         pe_en;
                
                // Activation: broadcast same value to all rows for a given column
                assign pe_ifmap = ifmap_data[c*DATA_WIDTH +: DATA_WIDTH];
                
                // Weight: unique per PE position
                assign pe_weight = weight_data[(r*PE_COLS + c)*DATA_WIDTH +: DATA_WIDTH];
                
                // Enable: product of global, row, and column enables
                assign pe_en = en & row_en[r] & col_en[c];
                
                pe #(
                    .DATA_WIDTH (DATA_WIDTH),
                    .ACC_WIDTH  (ACC_WIDTH)
                ) pe_inst (
                    .clk        (clk),
                    .rst        (rst),
                    .en         (pe_en),
                    .acc_clear  (acc_clear),
                    .ifmap_data (pe_ifmap),
                    .weight_data(pe_weight),
                    .acc_out    (acc_out[(r*PE_COLS + c)*ACC_WIDTH +: ACC_WIDTH])
                );
            end
        end
    endgenerate

endmodule
