/*
------------------------------------------------------------------------------
Module Name : camera_if
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    OV7670 DVP (Digital Video Port) camera interface.
    
    Functional Role:
    - Receives 8-bit parallel data from OV7670 sensor
    - Assembles two consecutive bytes into one RGB565 pixel
    - Detects frame boundaries via VSYNC
    - Detects line boundaries via HREF
    - Generates frame_start, frame_end, and pixel_valid signals
    - Detects frame errors (unexpected VSYNC, pixel count mismatch)
    
    Input/Output Contract:
    - Inputs clocked on PCLK domain (camera pixel clock)
    - Outputs synchronous to PCLK
    - pixel_data[15:0] valid only when pixel_valid is HIGH
    - frame_start pulses HIGH for one PCLK at start of frame
    - frame_end pulses HIGH for one PCLK at end of frame
    - frame_error asserted on protocol violations
    
    Data Widths:
    - D[7:0]: Raw camera data bus
    - pixel_data[15:0]: Assembled RGB565 pixel
    
    Timing:
    - OV7670 outputs data on rising edge of PCLK
    - Two PCLK cycles per RGB565 pixel (byte0 = [15:8], byte1 = [7:0])
    - HREF HIGH during active pixel region
    - VSYNC active-HIGH indicates vertical blanking
    
    Reset:
    - Synchronous active-HIGH reset
    - Clears all state and counters

Dataflow    :
    Output-Stationary CNN accelerator pipeline — camera capture stage

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module camera_if #(
    parameter IMG_W        = `IMG_WIDTH,
    parameter IMG_H        = `IMG_HEIGHT,
    parameter PIXEL_WIDTH  = `PIXEL_RGB565_WIDTH
)(
    // Camera DVP signals
    input  wire        pclk,           // Pixel clock from OV7670
    input  wire        rst,            // Synchronous reset (active HIGH)
    input  wire        vsync,          // Vertical sync (active HIGH = blanking)
    input  wire        href,           // Horizontal reference (active HIGH = valid data)
    input  wire [7:0]  d,              // 8-bit parallel data bus
    
    // Output interface
    output reg  [PIXEL_WIDTH-1:0] pixel_data,   // Assembled RGB565 pixel
    output reg                    pixel_valid,   // Pixel data valid strobe
    output reg                    frame_start,   // Frame start pulse
    output reg                    frame_end,     // Frame end pulse
    output reg                    frame_error    // Frame protocol error
);

    // ========================================================================
    // Internal signals
    // ========================================================================
    
    // Byte assembly
    reg [7:0]  byte_latch;       // First byte storage
    reg        byte_toggle;      // 0 = first byte, 1 = second byte
    
    // VSYNC edge detection
    reg        vsync_d;          // Delayed VSYNC for edge detect
    wire       vsync_rising;
    wire       vsync_falling;
    
    // Frame state
    reg        frame_active;     // Currently in active frame
    
    // Pixel counting for error detection
    reg [31:0] pixel_count;      // Pixels received in current frame
    
    localparam EXPECTED_PIXELS = IMG_W * IMG_H;
    
    // ========================================================================
    // VSYNC edge detection
    // ========================================================================
    assign vsync_rising  = vsync & ~vsync_d;
    assign vsync_falling = ~vsync & vsync_d;
    
    always @(posedge pclk) begin
        if (rst) begin
            vsync_d <= 1'b0;
        end else begin
            vsync_d <= vsync;
        end
    end
    
    // ========================================================================
    // Frame boundary detection
    // ========================================================================
    // OV7670: VSYNC HIGH = vertical blanking (between frames)
    //         VSYNC goes LOW  = frame data starts
    //         VSYNC goes HIGH = frame data ends
    
    always @(posedge pclk) begin
        if (rst) begin
            frame_start  <= 1'b0;
            frame_end    <= 1'b0;
            frame_active <= 1'b0;
            frame_error  <= 1'b0;
        end else begin
            // Default: clear pulse signals
            frame_start <= 1'b0;
            frame_end   <= 1'b0;
            frame_error <= 1'b0;
            
            if (vsync_falling) begin
                // VSYNC falling edge: frame starts
                frame_start  <= 1'b1;
                frame_active <= 1'b1;
            end
            
            if (vsync_rising && frame_active) begin
                // VSYNC rising edge: frame ends
                frame_end    <= 1'b1;
                frame_active <= 1'b0;
                
                // Check pixel count
                if (pixel_count != EXPECTED_PIXELS) begin
                    frame_error <= 1'b1;
                end
            end
        end
    end
    
    // ========================================================================
    // Byte assembly: two bytes -> one RGB565 pixel
    // ========================================================================
    // OV7670 sends MSB first: byte0 = pixel[15:8], byte1 = pixel[7:0]
    
    always @(posedge pclk) begin
        if (rst) begin
            byte_latch   <= 8'd0;
            byte_toggle  <= 1'b0;
            pixel_data   <= {PIXEL_WIDTH{1'b0}};
            pixel_valid  <= 1'b0;
            pixel_count  <= 32'd0;
        end else begin
            pixel_valid <= 1'b0;
            
            if (vsync_falling) begin
                // Reset counters at frame start
                byte_toggle <= 1'b0;
                pixel_count <= 32'd0;
            end
            
            if (href && frame_active) begin
                if (!byte_toggle) begin
                    // First byte: store MSB
                    byte_latch  <= d;
                    byte_toggle <= 1'b1;
                end else begin
                    // Second byte: assemble full pixel
                    pixel_data  <= {byte_latch, d};
                    pixel_valid <= 1'b1;
                    pixel_count <= pixel_count + 32'd1;
                    byte_toggle <= 1'b0;
                end
            end else begin
                // Reset toggle when HREF goes low (end of line)
                byte_toggle <= 1'b0;
            end
        end
    end

endmodule
