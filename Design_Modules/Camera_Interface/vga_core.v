`timescale 1ns / 1ps

/*
 * vga_core.v
 *
 * VGA timing generator for 640x480 resolution at 60Hz.
 * Generates horizontal and vertical sync, pixel coordinates, and active video signal.
 *
 * Inputs:
 * - clk   : pixel clock (25 MHz for 640x480 @ 60Hz)
 * - rst_n : active-low reset
 *
 * Outputs:
 * - hsync     : horizontal sync (active low)
 * - vsync     : vertical sync (active low)
 * - video_on  : high when within active display region
 * - pixel_x   : current pixel X coordinate
 * - pixel_y   : current pixel Y coordinate
 */

module vga_core(
    input  wire clk,
    input  wire rst_n,
    output wire hsync,
    output wire vsync,
    output reg  video_on,
    output wire [11:0] pixel_x,
    output wire [11:0] pixel_y
);

    // -------------------------------------------------------
    // VGA timing parameters for 640x480 @ 60Hz
    // -------------------------------------------------------
    localparam HD   = 640;  // Horizontal display area
    localparam HFP  = 16;   // Horizontal front porch
    localparam HSW  = 96;   // Horizontal sync pulse width
    localparam HBP  = 48;   // Horizontal back porch

    localparam VD   = 480;  // Vertical display area
    localparam VFP  = 10;   // Vertical front porch
    localparam VSW  = 2;    // Vertical sync pulse width
    localparam VBP  = 33;   // Vertical back porch

    localparam HTOTAL = HD + HFP + HSW + HBP;
    localparam VTOTAL = VD + VFP + VSW + VBP;

    // -------------------------------------------------------
    // Counters
    // -------------------------------------------------------
    reg [11:0] hctr_q, hctr_d;
    reg [11:0] vctr_q, vctr_d;

    // -------------------------------------------------------
    // Sync registers
    // -------------------------------------------------------
    reg hsync_q, hsync_d;
    reg vsync_q, vsync_d;

    // -------------------------------------------------------
    // Counter registers update
    // -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hctr_q   <= 0;
            vctr_q   <= 0;
            hsync_q  <= 1;
            vsync_q  <= 1;
        end else begin
            hctr_q   <= hctr_d;
            vctr_q   <= vctr_d;
            hsync_q  <= hsync_d;
            vsync_q  <= vsync_d;
        end
    end

    // -------------------------------------------------------
    // Next-state logic
    // -------------------------------------------------------
    always @* begin
        // Default assignments
        hctr_d    = hctr_q + 1;
        vctr_d    = vctr_q;
        hsync_d   = 1;
        vsync_d   = 1;
        video_on  = 0;

        // Horizontal counter wrap
        if (hctr_q == HTOTAL - 1) begin
            hctr_d = 0;
            // Vertical counter increment
            if (vctr_q == VTOTAL - 1)
                vctr_d = 0;
            else
                vctr_d = vctr_q + 1;
        end

        // Active video region
        if (hctr_q < HD && vctr_q < VD)
            video_on = 1;

        // Horizontal sync pulse (active low)
        if (hctr_q >= HD + HFP && hctr_q < HD + HFP + HSW)
            hsync_d = 0;

        // Vertical sync pulse (active low)
        if (vctr_q >= VD + VFP && vctr_q < VD + VFP + VSW)
            vsync_d = 0;
    end

    // -------------------------------------------------------
    // Output assignments
    // -------------------------------------------------------
    assign pixel_x = hctr_q;
    assign pixel_y = vctr_q;
    assign hsync   = hsync_q;
    assign vsync   = vsync_q;

endmodule
