`timescale 1ns / 1ps

/*
 * vga_interface.v
 *
 * Interface module between VGA display and asynchronous FIFO (camera input).
 * Handles reading pixel data from FIFO and outputting RGB and sync signals to VGA.
 *
 * Inputs:
 * - clk        : system clock
 * - rst_n      : active-low reset
 * - empty_fifo : FIFO empty flag
 * - din        : pixel data from FIFO (16-bit RGB)
 *
 * Outputs:
 * - clk_vga    : 25MHz VGA pixel clock
 * - rd_en      : read enable for FIFO
 * - vga_out_r  : red channel output (5-bit)
 * - vga_out_g  : green channel output (6-bit)
 * - vga_out_b  : blue channel output (5-bit)
 * - vga_out_vs : vertical sync
 * - vga_out_hs : horizontal sync
 */

module vga_interface(
    input  wire clk,
    input  wire rst_n,
    input  wire empty_fifo,
    input  wire [15:0] din,
    output wire clk_vga,
    output reg  rd_en,
    output reg [4:0] vga_out_r,
    output reg [5:0] vga_out_g,
    output reg [4:0] vga_out_b,
    output wire vga_out_vs,
    output wire vga_out_hs
);

    // FSM states
    localparam DELAY   = 2'b00,
               IDLE    = 2'b01,
               DISPLAY = 2'b10;

    reg [1:0] state_q = DELAY, state_d;

    // VGA pixel coordinates
    wire [11:0] pixel_x, pixel_y;

    // Clock for VGA
    wire clk_out;

    // ----------------------
    // FSM registers update
    // ----------------------
    always @(posedge clk_out or negedge rst_n) begin
        if (!rst_n)
            state_q <= DELAY;
        else
            state_q <= state_d;
    end

    // ----------------------
    // FSM next-state logic
    // ----------------------
    always @(*) begin
        // Defaults
        state_d     = state_q;
        rd_en       = 1'b0;
        vga_out_r   = 5'b0;
        vga_out_g   = 6'b0;
        vga_out_b   = 5'b0;

        case (state_q)
            DELAY: begin
                // Wait one frame (first pixel)
                if (pixel_x == 12'd1 && pixel_y == 12'd1)
                    state_d = IDLE;
            end

            IDLE: begin
                // Wait for FIFO data
                if (!empty_fifo && pixel_x == 12'd1 && pixel_y == 12'd0) begin
                    vga_out_r = din[15:11];
                    vga_out_g = din[10:5];
                    vga_out_b = din[4:0];
                    rd_en     = 1'b1;
                    state_d   = DISPLAY;
                end
            end

            DISPLAY: begin
                // Continue reading from FIFO while inside visible area
                if (pixel_x >= 12'd1 && pixel_x <= 12'd640 && pixel_y < 12'd480) begin
                    vga_out_r = din[15:11];
                    vga_out_g = din[10:5];
                    vga_out_b = din[4:0];
                    rd_en     = 1'b1;
                end else begin
                    state_d = DELAY; // return to DELAY at end of frame
                end
            end

            default: state_d = DELAY;
        endcase
    end

    assign clk_vga = clk_out;

    // ----------------------
    // Module instantiations
    // ----------------------
    vga_core u_vga_core (
        .clk(clk_out),
        .rst_n(rst_n),
        .hsync(vga_out_hs),
        .vsync(vga_out_vs),
        .video_on(),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y)
    );

    // Clock generation for VGA (25MHz)
    dcm_25MHz u_dcm_25MHz (
        .clk(clk),
        .clk_out(clk_out),
        .RESET(~rst_n),
        .LOCKED() // optional status
    );

endmodule
