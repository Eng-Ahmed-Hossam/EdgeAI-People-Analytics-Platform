/*
------------------------------------------------------------------------------
Module Name : cnn_input_adapter
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Bridge between SDRAM frame buffer and CNN ifmap buffer.
    
    Functional Role:
    - Reads RGB565 pixels from SDRAM and converts to INT8 channels
    - Loads first-layer input feature map into ifmap_buffer
    - Handles RGB565 → 3-channel INT8 conversion
    - Sequential read with address generation
    
    Input/Output Contract:
    - start: Begin loading a frame from SDRAM into ifmap_buffer
    - done: Loading complete
    - SDRAM read interface: rd_req, rd_addr, rd_data, rd_valid, rd_ack
    - ifmap write interface: wr_en, wr_addr, wr_data
    
    Data Widths:
    - SDRAM: 16-bit RGB565
    - ifmap: 8-bit INT8 per channel
    
    Timing:
    - One SDRAM read per pixel, 3 ifmap writes per pixel (R, G, B)
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — input adaptation

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module cnn_input_adapter #(
    parameter IMG_W         = `IMG_WIDTH,
    parameter IMG_H         = `IMG_HEIGHT,
    parameter DATA_WIDTH    = `DATA_WIDTH,
    parameter SDRAM_AW      = `SDRAM_ADDR_WIDTH,
    parameter SDRAM_DW      = `SDRAM_DATA_WIDTH,
    parameter BUF_AW        = $clog2(`MAX_IMG_DIM * `MAX_IMG_DIM)
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Control
    input  wire                   start,
    input  wire [SDRAM_AW-1:0]    frame_base_addr,   // SDRAM base address of frame
    output reg                    done,
    
    // SDRAM read interface
    output reg                    sdram_rd_req,
    output reg  [SDRAM_AW-1:0]   sdram_rd_addr,
    input  wire [SDRAM_DW-1:0]   sdram_rd_data,
    input  wire                   sdram_rd_valid,
    
    // ifmap buffer write interface (3 channels planar)
    output reg                    ifmap_wr_en,
    output reg  [BUF_AW-1:0]     ifmap_wr_addr,
    output reg  signed [DATA_WIDTH-1:0] ifmap_wr_data
);

    // ========================================================================
    // State machine
    // ========================================================================
    localparam S_IDLE       = 3'd0;
    localparam S_READ_REQ   = 3'd1;
    localparam S_READ_WAIT  = 3'd2;
    localparam S_WRITE_R    = 3'd3;
    localparam S_WRITE_G    = 3'd4;
    localparam S_WRITE_B    = 3'd5;
    localparam S_DONE       = 3'd6;
    
    reg [2:0] state;
    
    localparam TOTAL_PIXELS = IMG_W * IMG_H;
    localparam CHANNEL_SIZE = IMG_W * IMG_H;  // Pixels per channel plane
    
    reg [31:0]        pixel_idx;
    reg [SDRAM_AW-1:0] current_addr;
    reg [SDRAM_DW-1:0] pixel_latch;      // Latched RGB565 pixel
    
    // RGB565 extraction
    wire [4:0] r5;
    wire [5:0] g6;
    wire [4:0] b5;
    
    assign r5 = pixel_latch[15:11];
    assign g6 = pixel_latch[10:5];
    assign b5 = pixel_latch[4:0];
    
    // Scale to INT8 (0-255 range, then center to signed)
    // R: 5-bit → 8-bit: {r5, r5[4:2]}  (replicate MSBs)
    // G: 6-bit → 8-bit: {g6, g6[5:4]}
    // B: 5-bit → 8-bit: {b5, b5[4:2]}
    // Then convert to signed: subtract 128
    wire [7:0] r8, g8, b8;
    assign r8 = {r5, r5[4:2]};
    assign g8 = {g6, g6[5:4]};
    assign b8 = {b5, b5[4:2]};
    
    wire signed [DATA_WIDTH-1:0] r_int8, g_int8, b_int8;
    assign r_int8 = $signed({1'b0, r8[7:1]}) - 8'sd64;  // Rough centering
    assign g_int8 = $signed({1'b0, g8[7:1]}) - 8'sd64;
    assign b_int8 = $signed({1'b0, b8[7:1]}) - 8'sd64;
    
    // ========================================================================
    // Main FSM
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state        <= S_IDLE;
            done         <= 1'b0;
            sdram_rd_req <= 1'b0;
            sdram_rd_addr<= 0;
            ifmap_wr_en  <= 1'b0;
            ifmap_wr_addr<= 0;
            ifmap_wr_data<= 0;
            pixel_idx    <= 0;
            current_addr <= 0;
            pixel_latch  <= 0;
        end else begin
            // Defaults
            done         <= 1'b0;
            sdram_rd_req <= 1'b0;
            ifmap_wr_en  <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    if (start) begin
                        pixel_idx    <= 0;
                        current_addr <= frame_base_addr;
                        state        <= S_READ_REQ;
                    end
                end
                
                S_READ_REQ: begin
                    sdram_rd_req  <= 1'b1;
                    sdram_rd_addr <= current_addr;
                    state         <= S_READ_WAIT;
                end
                
                S_READ_WAIT: begin
                    if (sdram_rd_valid) begin
                        pixel_latch <= sdram_rd_data;
                        state       <= S_WRITE_R;
                    end
                end
                
                S_WRITE_R: begin
                    // Write R channel (plane 0)
                    ifmap_wr_en   <= 1'b1;
                    ifmap_wr_addr <= pixel_idx[BUF_AW-1:0];
                    ifmap_wr_data <= r_int8;
                    state         <= S_WRITE_G;
                end
                
                S_WRITE_G: begin
                    // Write G channel (plane 1)
                    ifmap_wr_en   <= 1'b1;
                    ifmap_wr_addr <= pixel_idx[BUF_AW-1:0] + CHANNEL_SIZE;
                    ifmap_wr_data <= g_int8;
                    state         <= S_WRITE_B;
                end
                
                S_WRITE_B: begin
                    // Write B channel (plane 2)
                    ifmap_wr_en   <= 1'b1;
                    ifmap_wr_addr <= pixel_idx[BUF_AW-1:0] + 2 * CHANNEL_SIZE;
                    ifmap_wr_data <= b_int8;
                    
                    pixel_idx    <= pixel_idx + 1;
                    current_addr <= current_addr + 1;
                    
                    if (pixel_idx == TOTAL_PIXELS - 1) begin
                        state <= S_DONE;
                    end else begin
                        state <= S_READ_REQ;
                    end
                end
                
                S_DONE: begin
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
