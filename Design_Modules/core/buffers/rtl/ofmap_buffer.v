/*
------------------------------------------------------------------------------
Module Name : ofmap_buffer
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Output feature map buffer for CNN core.
    
    Functional Role:
    - Stores output feature map for current layer
    - Written by post-processing pipeline
    - Becomes ifmap for next layer (via address swap)
    - Supports sequential write and random read
    
    Input/Output Contract:
    - Write port: wr_en, wr_addr, wr_data
    - Read port: rd_en, rd_addr, rd_data, rd_valid
    - Parameterized buffer size
    
    Data Widths:
    - Data: DATA_WIDTH (8-bit)
    - Address: ADDR_WIDTH derived from buffer size
    
    Timing:
    - 1 clock cycle read latency
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — output storage

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module ofmap_buffer #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter BUF_DEPTH  = `MAX_IMG_DIM * `MAX_IMG_DIM,
    parameter ADDR_WIDTH = $clog2(BUF_DEPTH)
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Write port (from post-processing pipeline)
    input  wire                   wr_en,
    input  wire [ADDR_WIDTH-1:0]  wr_addr,
    input  wire signed [DATA_WIDTH-1:0] wr_data,
    
    // Read port (for next layer or tensor output)
    input  wire                   rd_en,
    input  wire [ADDR_WIDTH-1:0]  rd_addr,
    output reg  signed [DATA_WIDTH-1:0] rd_data,
    output reg                    rd_valid
);

    // ========================================================================
    // Buffer memory
    // ========================================================================
    reg signed [DATA_WIDTH-1:0] mem [0:BUF_DEPTH-1];
    
    // ========================================================================
    // Write logic
    // ========================================================================
    always @(posedge clk) begin
        if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end
    
    // ========================================================================
    // Read logic (1 cycle latency)
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            rd_data  <= 0;
            rd_valid <= 1'b0;
        end else begin
            rd_valid <= rd_en;
            if (rd_en) begin
                rd_data <= mem[rd_addr];
            end
        end
    end

endmodule
