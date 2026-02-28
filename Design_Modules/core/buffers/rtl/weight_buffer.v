/*
------------------------------------------------------------------------------
Module Name : weight_buffer
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Double-buffered weight storage for CNN core.
    
    Functional Role:
    - Stores INT8 weights for current and next layers
    - NVDLA-style double buffering: preload next while computing current
    - Feeds PE array columns with weight values
    - Initialized from .hex files in simulation
    
    Input/Output Contract:
    - Load port: ld_en, ld_addr, ld_data, ld_bank (select fill bank)
    - Read port: rd_en, rd_addr, rd_data, rd_bank (select active bank)
    - swap: Swap active/fill banks
    
    Data Widths:
    - Data: DATA_WIDTH (8-bit)
    - Address: ADDR_WIDTH for max weight storage
    
    Timing:
    - 1 clock cycle read latency
    - Bank swap is instantaneous (pointer flip)
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — weight storage (NVDLA-style)

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module weight_buffer #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    // Max weights per layer: 256 output ch × 256 input ch × 3 × 3 = 589824
    // We use a smaller practical limit to save BRAM
    parameter BUF_DEPTH  = 65536,  // 64K entries per bank
    parameter ADDR_WIDTH = $clog2(BUF_DEPTH)
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Load port (fill bank — for preloading next layer)
    input  wire                   ld_en,
    input  wire [ADDR_WIDTH-1:0]  ld_addr,
    input  wire signed [DATA_WIDTH-1:0] ld_data,
    
    // Read port (active bank — feeding PE array)
    input  wire                   rd_en,
    input  wire [ADDR_WIDTH-1:0]  rd_addr,
    output reg  signed [DATA_WIDTH-1:0] rd_data,
    output reg                    rd_valid,
    
    // Bank control
    input  wire                   swap       // Swap active and fill banks
);

    // ========================================================================
    // Double-buffered weight banks
    // ========================================================================
    reg signed [DATA_WIDTH-1:0] bank_a [0:BUF_DEPTH-1];
    reg signed [DATA_WIDTH-1:0] bank_b [0:BUF_DEPTH-1];
    
    reg active_bank;  // 0 = read from A, load to B; 1 = read from B, load to A
    
    // ========================================================================
    // Bank swap
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            active_bank <= 1'b0;
        end else if (swap) begin
            active_bank <= ~active_bank;
        end
    end
    
    // ========================================================================
    // Load logic (write to fill bank)
    // ========================================================================
    always @(posedge clk) begin
        if (ld_en) begin
            if (active_bank == 1'b0) begin
                // Active = A, so fill B
                bank_b[ld_addr] <= ld_data;
            end else begin
                // Active = B, so fill A
                bank_a[ld_addr] <= ld_data;
            end
        end
    end
    
    // ========================================================================
    // Read logic (read from active bank)
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            rd_data  <= 0;
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= rd_en;
            if (rd_en) begin
                if (active_bank == 1'b0) begin
                    rd_data <= bank_a[rd_addr];
                end else begin
                    rd_data <= bank_b[rd_addr];
                end
            end
        end
    end

endmodule
