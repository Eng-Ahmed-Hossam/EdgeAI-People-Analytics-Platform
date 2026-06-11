/*
------------------------------------------------------------------------------
Module Name : pixel_fifo
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Asynchronous FIFO for clock domain crossing between camera PCLK
    and system clock domains.
    
    Functional Role:
    - Buffers pixel data from camera interface (PCLK domain)
    - Provides readback in system clock domain (SYS_CLK)
    - Uses Gray-coded pointers for safe CDC
    
    Input/Output Contract:
    - wr_clk/wr_en/wr_data: Write port (camera PCLK domain)
    - rd_clk/rd_en/rd_data: Read port (system clock domain)
    - full/empty: Status flags (in respective clock domains)
    
    Data Widths:
    - Parameterized DATA_WIDTH (default 16 for RGB565)
    - Parameterized ADDR_WIDTH (determines depth = 2^ADDR_WIDTH)
    
    Timing:
    - Write data captured on wr_clk rising edge when wr_en & ~full
    - Read data available on rd_clk rising edge when rd_en & ~empty
    - Gray-code synchronizers use 2-FF chain
    
    Reset:
    - Asynchronous active-HIGH reset (both domains)

Dataflow    :
    Output-Stationary CNN accelerator pipeline — pixel buffering stage

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pixel_fifo #(
    parameter DATA_WIDTH = `PIXEL_RGB565_WIDTH,
    parameter ADDR_WIDTH = `PIXEL_FIFO_ADDR_W
)(
    // Write port (camera PCLK domain)
    input  wire                  wr_clk,
    input  wire                  wr_rst,
    input  wire                  wr_en,
    input  wire [DATA_WIDTH-1:0] wr_data,
    output wire                  full,
    output wire                  almost_full,
    
    // Read port (system clock domain)
    input  wire                  rd_clk,
    input  wire                  rd_rst,
    input  wire                  rd_en,
    output wire [DATA_WIDTH-1:0] rd_data,
    output wire                  empty
);

    // ========================================================================
    // Dual-port RAM
    // ========================================================================
    localparam DEPTH = 1 << ADDR_WIDTH;
    localparam AF_THRESHOLD = DEPTH - 4;  // Almost full when 4 slots remain
    
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    
    // ========================================================================
    // Write and read pointers (binary + Gray)
    // ========================================================================
    reg [ADDR_WIDTH:0] wr_ptr_bin;     // Extra bit for full/empty detection
    reg [ADDR_WIDTH:0] rd_ptr_bin;
    
    wire [ADDR_WIDTH:0] wr_ptr_gray;
    wire [ADDR_WIDTH:0] rd_ptr_gray;
    
    // Gray code conversion
    assign wr_ptr_gray = wr_ptr_bin ^ (wr_ptr_bin >> 1);
    assign rd_ptr_gray = rd_ptr_bin ^ (rd_ptr_bin >> 1);
    
    // ========================================================================
    // CDC: Synchronize Gray-coded pointers across domains (2-FF)
    // ========================================================================
    reg [ADDR_WIDTH:0] wr_ptr_gray_sync1, wr_ptr_gray_sync2;  // wr→rd domain
    reg [ADDR_WIDTH:0] rd_ptr_gray_sync1, rd_ptr_gray_sync2;  // rd→wr domain
    
    // Synchronize write pointer into read domain
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            wr_ptr_gray_sync1 <= 0;
            wr_ptr_gray_sync2 <= 0;
        end else begin
            wr_ptr_gray_sync1 <= wr_ptr_gray;
            wr_ptr_gray_sync2 <= wr_ptr_gray_sync1;
        end
    end
    
    // Synchronize read pointer into write domain
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            rd_ptr_gray_sync1 <= 0;
            rd_ptr_gray_sync2 <= 0;
        end else begin
            rd_ptr_gray_sync1 <= rd_ptr_gray;
            rd_ptr_gray_sync2 <= rd_ptr_gray_sync1;
        end
    end
    
    // ========================================================================
    // Full and empty detection
    // ========================================================================
    // Full: Gray codes differ in top 2 bits, same in rest
    assign full = (wr_ptr_gray[ADDR_WIDTH]     != rd_ptr_gray_sync2[ADDR_WIDTH]) &&
                  (wr_ptr_gray[ADDR_WIDTH-1]   != rd_ptr_gray_sync2[ADDR_WIDTH-1]) &&
                  (wr_ptr_gray[ADDR_WIDTH-2:0] == rd_ptr_gray_sync2[ADDR_WIDTH-2:0]);
    
    // Empty: Gray codes are identical
    assign empty = (wr_ptr_gray_sync2 == rd_ptr_gray);
    
    // Almost full: based on binary pointer difference in write domain
    // Convert synced read gray back to binary for comparison
    wire [ADDR_WIDTH:0] rd_ptr_bin_in_wr;
    
    // Gray-to-binary conversion for synced read pointer
    genvar gi;
    wire [ADDR_WIDTH:0] rd_gray_to_bin;
    assign rd_gray_to_bin[ADDR_WIDTH] = rd_ptr_gray_sync2[ADDR_WIDTH];
    generate
        for (gi = ADDR_WIDTH - 1; gi >= 0; gi = gi - 1) begin : gray2bin
            assign rd_gray_to_bin[gi] = rd_gray_to_bin[gi+1] ^ rd_ptr_gray_sync2[gi];
        end
    endgenerate
    
    wire [ADDR_WIDTH:0] fill_level;
    assign fill_level = wr_ptr_bin - rd_gray_to_bin;
    assign almost_full = (fill_level >= AF_THRESHOLD);
    
    // ========================================================================
    // Write logic (wr_clk domain)
    // ========================================================================
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wr_ptr_bin <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
                wr_ptr_bin <= wr_ptr_bin + 1;
            end
        end
    end
    
    // ========================================================================
    // Read logic (rd_clk domain)
    // ========================================================================
    reg [DATA_WIDTH-1:0] rd_data_reg;
    
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rd_ptr_bin  <= 0;
            rd_data_reg <= 0;
        end else begin
            if (rd_en && !empty) begin
                rd_data_reg <= mem[rd_ptr_bin[ADDR_WIDTH-1:0]];
                rd_ptr_bin  <= rd_ptr_bin + 1;
            end
        end
    end
    
    assign rd_data = rd_data_reg;

endmodule
