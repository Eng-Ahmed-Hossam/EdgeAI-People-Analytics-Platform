/*
 * asyn_fifo.v
 *
 * Asynchronous FIFO (Dual-Clock FIFO)
 *
 * This module implements a dual-clock FIFO buffer used to transfer data between
 * two asynchronous clock domains. It uses Gray-coded read/write pointers and
 * pointer synchronization to avoid metastability and ensure safe operation.
 *
 * Parameters:
 * DATA_WIDTH       – width of each data word stored in FIFO
 * FIFO_DEPTH_WIDTH – number of address bits (FIFO depth = 2^FIFO_DEPTH_WIDTH)
 *
 * Inputs:
 * - rst_n       : active-low reset
 * - clk_write   : write-clock domain
 * - clk_read    : read-clock domain
 * - write       : write enable (increment write pointer when !full)
 * - read        : read enable  (increment read pointer when !empty)
 * - data_write  : data input
 *
 * Outputs:
 * - data_read   : data output
 * - full        : FIFO full flag (write-side domain)
 * - empty       : FIFO empty flag (read-side domain)
 * - data_count_w: number of stored words (write-clock domain view)
 * - data_count_r: number of stored words (read-clock domain view)
 */

`timescale 1ns/1ps

module asyn_fifo #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH_WIDTH = 11     // FIFO depth = 2^11 = 2048
)(
    input  wire                         rst_n,
    input  wire                         clk_write,
    input  wire                         clk_read,
    input  wire                         write,
    input  wire                         read,
    input  wire [DATA_WIDTH-1:0]        data_write,
    output wire [DATA_WIDTH-1:0]        data_read,
    output reg                          full,
    output reg                          empty,
    output reg [FIFO_DEPTH_WIDTH:0]     data_count_w,
    output reg [FIFO_DEPTH_WIDTH:0]     data_count_r
);

    localparam FIFO_DEPTH = (1 << FIFO_DEPTH_WIDTH);

    // -----------------------------------------------------------------------------
    // Pointer registers (one bit wider than address to detect wrapping)
    // -----------------------------------------------------------------------------
    reg [FIFO_DEPTH_WIDTH:0] w_ptr_q;
    reg [FIFO_DEPTH_WIDTH:0] r_ptr_q;

    // Gray-coded pointers
    wire [FIFO_DEPTH_WIDTH:0] w_grey     = w_ptr_q ^ (w_ptr_q >> 1);
    wire [FIFO_DEPTH_WIDTH:0] w_grey_nxt = (w_ptr_q + 1'b1) ^ ((w_ptr_q + 1'b1) >> 1);

    wire [FIFO_DEPTH_WIDTH:0] r_grey     = r_ptr_q ^ (r_ptr_q >> 1);
    wire [FIFO_DEPTH_WIDTH:0] r_grey_nxt = (r_ptr_q + 1'b1) ^ ((r_ptr_q + 1'b1) >> 1);

    wire we = write && !full;   // write enable when FIFO not full

    // -----------------------------------------------------------------------------
    // Synchronizers (2-stage) for pointer crossing
    // -----------------------------------------------------------------------------

    // read → write domain
    reg [FIFO_DEPTH_WIDTH:0] r_grey_sync, r_grey_sync_temp;

    // write → read domain
    reg [FIFO_DEPTH_WIDTH:0] w_grey_sync, w_grey_sync_temp;

    always @(posedge clk_write or negedge rst_n) begin
        if (!rst_n) begin
            r_grey_sync_temp <= 0;
            r_grey_sync      <= 0;
        end else begin
            r_grey_sync_temp <= r_grey;
            r_grey_sync      <= r_grey_sync_temp;
        end
    end

    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            w_grey_sync_temp <= 0;
            w_grey_sync      <= 0;
        end else begin
            w_grey_sync_temp <= w_grey;
            w_grey_sync      <= w_grey_sync_temp;
        end
    end

    // -----------------------------------------------------------------------------
    // Gray → Binary conversion function
    // -----------------------------------------------------------------------------
    function [FIFO_DEPTH_WIDTH:0] grey_to_bin;
        input [FIFO_DEPTH_WIDTH:0] grey;
        integer i;
    begin
        grey_to_bin[FIFO_DEPTH_WIDTH] = grey[FIFO_DEPTH_WIDTH];
        for (i = FIFO_DEPTH_WIDTH-1; i >= 0; i = i - 1)
            grey_to_bin[i] = grey_to_bin[i+1] ^ grey[i];
    end
    endfunction

    // Synchronized binary pointers
    reg [FIFO_DEPTH_WIDTH:0] r_ptr_sync_bin;
    reg [FIFO_DEPTH_WIDTH:0] w_ptr_sync_bin;

    always @(*) begin
        r_ptr_sync_bin = grey_to_bin(r_grey_sync);
        w_ptr_sync_bin = grey_to_bin(w_grey_sync);
    end

    // -----------------------------------------------------------------------------
    // WRITE DOMAIN LOGIC
    // -----------------------------------------------------------------------------
    always @(posedge clk_write or negedge rst_n) begin
        if (!rst_n) begin
            w_ptr_q     <= 0;
            full        <= 1'b0;
            data_count_w<= 0;
        end else begin
            
            if (we)
                w_ptr_q <= w_ptr_q + 1'b1;

            // Full detection
            full <= (we ? w_grey_nxt : w_grey) ==
                    {~r_grey_sync[FIFO_DEPTH_WIDTH],
                    r_grey_sync[FIFO_DEPTH_WIDTH-1:0]};

            // Data count (write-side view)
            if (w_ptr_q >= r_ptr_sync_bin)
                data_count_w <= w_ptr_q - r_ptr_sync_bin;
            else
                data_count_w <= FIFO_DEPTH - r_ptr_sync_bin + w_ptr_q;
        end
    end

    // -----------------------------------------------------------------------------
    // READ DOMAIN LOGIC
    // -----------------------------------------------------------------------------
    wire [FIFO_DEPTH_WIDTH:0] r_ptr_d =
        (read && !empty) ? (r_ptr_q + 1'b1) : r_ptr_q;

    always @(posedge clk_read or negedge rst_n) begin
        if (!rst_n) begin
            r_ptr_q     <= 0;
            empty       <= 1'b1;
            data_count_r<= 0;
        end else begin
            r_ptr_q <= r_ptr_d;

            // Empty detection
            empty <= (read && !empty) ?
                    (r_grey_nxt == w_grey_sync) :
                    (r_grey     == w_grey_sync);

            // Data count (read-side view)
            if (w_ptr_sync_bin >= r_ptr_q)
                data_count_r <= w_ptr_sync_bin - r_ptr_q;
            else
                data_count_r <= FIFO_DEPTH - r_ptr_q + w_ptr_sync_bin;
        end
    end

    // -----------------------------------------------------------------------------
    // Dual-Port RAM (synchronous)
    // -----------------------------------------------------------------------------
    dual_port_sync #(
        .ADDR_WIDTH(FIFO_DEPTH_WIDTH),
        .DATA_WIDTH(DATA_WIDTH)
    ) ram_inst (
        .clk_r(clk_read),
        .clk_w(clk_write),
        .we(we),
        .din(data_write),
        .addr_a(w_ptr_q[FIFO_DEPTH_WIDTH-1:0]),
        .addr_b(r_ptr_d[FIFO_DEPTH_WIDTH-1:0]),
        .dout(data_read)
    );

endmodule


/* ============================================================================
 * Dual-Port Synchronous RAM
 * Used as the storage buffer for the FIFO.
 * Address A = write port, Address B = read port.
 * ============================================================================
 */
module dual_port_sync #(
    parameter ADDR_WIDTH = 11,
    parameter DATA_WIDTH = 8
)(
    input                     clk_r,
    input                     clk_w,
    input                     we,
    input  [DATA_WIDTH-1:0]   din,
    input  [ADDR_WIDTH-1:0]   addr_a, addr_b,
    output [DATA_WIDTH-1:0]   dout
);

    reg [DATA_WIDTH-1:0] ram[(1<<ADDR_WIDTH)-1:0];
    reg [ADDR_WIDTH-1:0] addr_b_q;

    always @(posedge clk_w) begin
        if (we)
            ram[addr_a] <= din;
    end

    always @(posedge clk_r) begin
        addr_b_q <= addr_b;
    end

    assign dout = ram[addr_b_q];

endmodule
