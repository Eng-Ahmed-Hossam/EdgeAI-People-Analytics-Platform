/*
------------------------------------------------------------------------------
Module Name : frame_buffer_mgr
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Ping-pong frame buffer manager for SDRAM.
    
    Functional Role:
    - Manages two frame buffer regions in SDRAM (A and B)
    - Alternates write and read buffers between frames
    - Ensures CNN only reads from complete frames
    - Prevents read/write conflicts
    
    Input/Output Contract:
    - wr_done: Pulse when frame_writer completes a frame
    - rd_done: Pulse when CNN completes reading a frame
    - wr_base_addr: Current write buffer base address
    - rd_base_addr: Current read buffer base address
    - frame_ready: Asserted when a complete frame is available
    
    Data Widths:
    - Address: SDRAM_ADDR_WIDTH bits
    
    Timing:
    - Frame swap occurs on wr_done when previous read is complete
    - frame_ready deasserts during swap
    
    Reset:
    - Synchronous active-HIGH reset
    - Write starts at Buffer A, read at Buffer B

Dataflow    :
    Output-Stationary CNN accelerator pipeline — buffer management

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module frame_buffer_mgr #(
    parameter ADDR_WIDTH  = `SDRAM_ADDR_WIDTH,
    parameter BUF_A_BASE  = `FRAME_BUF_A_BASE,
    parameter BUF_B_BASE  = `FRAME_BUF_B_BASE
)(
    input  wire                  clk,
    input  wire                  rst,
    
    // Frame writer signals
    input  wire                  wr_done,        // Frame write complete
    output reg  [ADDR_WIDTH-1:0] wr_base_addr,   // Current write buffer base
    
    // CNN reader signals
    input  wire                  rd_done,         // CNN read complete
    output reg  [ADDR_WIDTH-1:0] rd_base_addr,   // Current read buffer base
    output reg                   frame_ready,     // Complete frame available
    
    // Status
    output reg                   buf_sel          // Current write buffer (0=A, 1=B)
);

    // ========================================================================
    // Ping-pong state
    // ========================================================================
    reg frame_written;    // A frame has been written and not yet read
    
    always @(posedge clk) begin
        if (rst) begin
            buf_sel       <= 1'b0;          // Start writing to buffer A
            wr_base_addr  <= BUF_A_BASE;
            rd_base_addr  <= BUF_B_BASE;
            frame_ready   <= 1'b0;
            frame_written <= 1'b0;
        end else begin
            // Frame write complete: mark frame as available
            if (wr_done) begin
                frame_written <= 1'b1;
                frame_ready   <= 1'b1;
            end
            
            // CNN read complete: swap buffers
            if (rd_done && frame_written) begin
                frame_written <= 1'b0;
                frame_ready   <= 1'b0;
                
                // Swap buffer assignments
                buf_sel <= ~buf_sel;
                if (buf_sel) begin
                    // Was writing to B, now write to A
                    wr_base_addr <= BUF_A_BASE;
                    rd_base_addr <= BUF_B_BASE;
                end else begin
                    // Was writing to A, now write to B
                    wr_base_addr <= BUF_B_BASE;
                    rd_base_addr <= BUF_A_BASE;
                end
            end
        end
    end

endmodule
