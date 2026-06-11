/*
------------------------------------------------------------------------------
Module Name : frame_writer
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Writes captured frame data from pixel FIFO to SDRAM.
    
    Functional Role:
    - Reads pixels from pixel_fifo (system clock domain)
    - Assembles burst-aligned write transactions
    - Tracks row/column position within frame
    - Issues write requests to sdram_ctrl
    
    Input/Output Contract:
    - fifo_rd_en/fifo_data/fifo_empty: Read interface to pixel_fifo
    - sdram_wr_req/sdram_wr_addr/sdram_wr_data/sdram_wr_ack: SDRAM write IF
    - frame_start: Begin capturing a new frame
    - write_done: Asserted when full frame is written to SDRAM
    
    Data Widths:
    - Pixel: 16-bit RGB565
    - SDRAM address: 24-bit
    - SDRAM data: 16-bit
    
    Timing:
    - Operates in system clock domain
    - Burst-aligned write sequences
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — frame write stage

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module frame_writer #(
    parameter IMG_W          = `IMG_WIDTH,
    parameter IMG_H          = `IMG_HEIGHT,
    parameter PIXEL_WIDTH    = `PIXEL_RGB565_WIDTH,
    parameter ADDR_WIDTH     = `SDRAM_ADDR_WIDTH,
    parameter DATA_WIDTH     = `SDRAM_DATA_WIDTH,
    parameter BURST_LEN      = `SDRAM_BURST_LEN
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Control
    input  wire                   frame_start,    // Pulse: start of new frame
    input  wire [ADDR_WIDTH-1:0]  base_addr,      // Base address for this frame
    output reg                    write_done,     // Frame write complete
    
    // Pixel FIFO read interface
    output reg                    fifo_rd_en,
    input  wire [PIXEL_WIDTH-1:0] fifo_data,
    input  wire                   fifo_empty,
    
    // SDRAM write interface
    output reg                    sdram_wr_req,
    output reg  [ADDR_WIDTH-1:0]  sdram_wr_addr,
    output reg  [DATA_WIDTH-1:0]  sdram_wr_data,
    input  wire                   sdram_wr_ack
);

    // ========================================================================
    // State machine
    // ========================================================================
    localparam S_IDLE       = 3'd0;
    localparam S_WAIT_DATA  = 3'd1;
    localparam S_READ_FIFO  = 3'd2;
    localparam S_WRITE_REQ  = 3'd3;
    localparam S_WRITE_WAIT = 3'd4;
    localparam S_DONE       = 3'd5;
    
    reg [2:0] state;
    
    // Pixel tracking
    localparam TOTAL_PIXELS = IMG_W * IMG_H;
    reg [31:0] pixel_idx;
    reg [ADDR_WIDTH-1:0] curr_addr;
    
    // Burst assembly
    reg [DATA_WIDTH-1:0] burst_buf [0:BURST_LEN-1];
    reg [$clog2(BURST_LEN):0] burst_cnt;
    reg [$clog2(BURST_LEN):0] burst_wr_idx;
    
    // ========================================================================
    // Main FSM
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            write_done   <= 1'b0;
            fifo_rd_en   <= 1'b0;
            sdram_wr_req <= 1'b0;
            sdram_wr_addr<= 0;
            sdram_wr_data<= 0;
            pixel_idx    <= 0;
            curr_addr    <= 0;
            burst_cnt    <= 0;
            burst_wr_idx <= 0;
        end else begin
            // Defaults
            fifo_rd_en   <= 1'b0;
            sdram_wr_req <= 1'b0;
            write_done   <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    if (frame_start) begin
                        pixel_idx  <= 0;
                        curr_addr  <= base_addr;
                        burst_cnt  <= 0;
                        state      <= S_WAIT_DATA;
                    end
                end
                
                S_WAIT_DATA: begin
                    if (pixel_idx >= TOTAL_PIXELS) begin
                        // All pixels written — flush remaining burst
                        if (burst_cnt > 0) begin
                            burst_wr_idx <= 0;
                            state        <= S_WRITE_REQ;
                        end else begin
                            state <= S_DONE;
                        end
                    end else if (!fifo_empty) begin
                        fifo_rd_en <= 1'b1;
                        state      <= S_READ_FIFO;
                    end
                end
                
                S_READ_FIFO: begin
                    // FIFO data available after rd_en
                    burst_buf[burst_cnt] <= fifo_data;
                    burst_cnt <= burst_cnt + 1;
                    pixel_idx <= pixel_idx + 1;
                    
                    if (burst_cnt == BURST_LEN - 1) begin
                        // Burst buffer full, write to SDRAM
                        burst_wr_idx <= 0;
                        state        <= S_WRITE_REQ;
                    end else begin
                        state <= S_WAIT_DATA;
                    end
                end
                
                S_WRITE_REQ: begin
                    sdram_wr_req  <= 1'b1;
                    sdram_wr_addr <= curr_addr;
                    sdram_wr_data <= burst_buf[burst_wr_idx];
                    state         <= S_WRITE_WAIT;
                end
                
                S_WRITE_WAIT: begin
                    if (sdram_wr_ack) begin
                        sdram_wr_req <= 1'b0;
                        curr_addr    <= curr_addr + 1;
                        burst_wr_idx <= burst_wr_idx + 1;
                        
                        if (burst_wr_idx == burst_cnt - 1) begin
                            // Burst complete
                            burst_cnt <= 0;
                            if (pixel_idx >= TOTAL_PIXELS) begin
                                state <= S_DONE;
                            end else begin
                                state <= S_WAIT_DATA;
                            end
                        end else begin
                            state <= S_WRITE_REQ;
                        end
                    end
                end
                
                S_DONE: begin
                    write_done <= 1'b1;
                    state      <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
