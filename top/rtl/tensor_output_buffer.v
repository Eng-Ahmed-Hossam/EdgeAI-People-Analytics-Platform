/*
------------------------------------------------------------------------------
Module Name : tensor_output_buffer
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Final detection tensor output buffer.
    
    Functional Role:
    - Stores the final YOLO detection tensor
    - Receives output from last CNN layer
    - Grid_size × Grid_size × Detect_channels = 13 × 13 × 255 bytes
    - Provides read interface for PS to consume detection results
    
    Input/Output Contract:
    - Write: wr_en, wr_data from CNN output (sequential)
    - Read: rd_en, rd_addr, rd_data for PS access
    - tensor_ready: Flag when full tensor is available
    - tensor_clear: Reset for new inference
    
    Data Widths:
    - Data: DATA_WIDTH (8-bit)
    - Address: log2(TENSOR_SIZE)
    
    Timing:
    - 1 clock cycle read latency
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — final output

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module tensor_output_buffer #(
    parameter DATA_WIDTH   = `DATA_WIDTH,
    parameter TENSOR_DEPTH = `GRID_SIZE * `GRID_SIZE * `DETECT_CHANNELS,
    parameter ADDR_WIDTH   = $clog2(TENSOR_DEPTH)
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Write port (from CNN last layer)
    input  wire                   wr_en,
    input  wire signed [DATA_WIDTH-1:0] wr_data,
    input  wire                   tensor_clear,    // Start new tensor
    
    // Read port (PS access)
    input  wire                   rd_en,
    input  wire [ADDR_WIDTH-1:0]  rd_addr,
    output reg  signed [DATA_WIDTH-1:0] rd_data,
    output reg                    rd_valid,
    
    // Status
    output reg                    tensor_ready     // Full tensor available
);

    // ========================================================================
    // Buffer memory
    // ========================================================================
    reg signed [DATA_WIDTH-1:0] mem [0:TENSOR_DEPTH-1];
    
    // Write pointer (auto-incrementing)
    reg [ADDR_WIDTH-1:0] wr_ptr;
    
    // ========================================================================
    // Write logic
    // ========================================================================
    always @(posedge clk) begin
        if (rst || tensor_clear) begin
            wr_ptr       <= 0;
            tensor_ready <= 1'b0;
        end else if (wr_en) begin
            mem[wr_ptr] <= wr_data;
            
            if (wr_ptr == TENSOR_DEPTH - 1) begin
                tensor_ready <= 1'b1;
                wr_ptr       <= 0;
            end else begin
                wr_ptr <= wr_ptr + 1;
            end
        end
    end
    
    // ========================================================================
    // Read logic
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
