/*
------------------------------------------------------------------------------
Module Name : requantizer
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Fixed-point requantization from INT32 accumulator to INT8 output.
    
    Functional Role:
    - Implements: scaled = (acc * multiplier) >> shift
    - Adds zero_point offset
    - Clamps result to INT8 range [-128, +127]
    - Per-channel multiplier, shift, and zero_point
    - Bit-accurate with Python golden model
    
    Input/Output Contract:
    - data_in: Signed INT32 (biased accumulator value)
    - multiplier: Unsigned fixed-point scale factor
    - shift: Right-shift amount
    - zero_point: Output zero point (signed INT8)
    - data_out: Requantized INT8 result
    - in_valid/out_valid: Pipeline valid handshake
    
    Data Widths:
    - Input: ACC_WIDTH (32-bit)
    - Multiplier: MULT_WIDTH (16-bit)
    - Internal product: ACC_WIDTH + MULT_WIDTH (48-bit)
    - Output: DATA_WIDTH (8-bit)
    
    Timing:
    - 2 clock cycle latency (pipeline registered)
    - Stage 1: Multiply
    - Stage 2: Shift + clamp
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — requantization

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module requantizer #(
    parameter ACC_WIDTH   = `ACC_WIDTH,
    parameter DATA_WIDTH  = `DATA_WIDTH,
    parameter MULT_WIDTH  = `MULT_WIDTH,
    parameter SHIFT_WIDTH = `SHIFT_WIDTH
)(
    input  wire                           clk,
    input  wire                           rst,
    
    // Input (from bias_add)
    input  wire signed [ACC_WIDTH-1:0]    data_in,
    input  wire                           in_valid,
    
    // Requantization parameters (per-channel)
    input  wire [MULT_WIDTH-1:0]          multiplier,  // Unsigned scale
    input  wire [SHIFT_WIDTH-1:0]         shift,       // Right shift amount
    input  wire signed [DATA_WIDTH-1:0]   zero_point,  // Output zero point
    
    // Output
    output reg  signed [DATA_WIDTH-1:0]   data_out,
    output reg                            out_valid
);

    // ========================================================================
    // Stage 1: Multiply
    // ========================================================================
    localparam PRODUCT_WIDTH = ACC_WIDTH + MULT_WIDTH;
    
    reg signed [PRODUCT_WIDTH-1:0] product_reg;
    reg [SHIFT_WIDTH-1:0]          shift_reg;
    reg signed [DATA_WIDTH-1:0]    zp_reg;
    reg                            valid_s1;
    
    // Use signed multiplication by extending multiplier with 0 MSB
    wire signed [MULT_WIDTH:0] mult_signed;
    assign mult_signed = {1'b0, multiplier};
    
    always @(posedge clk) begin
        if (rst) begin
            product_reg <= 0;
            shift_reg   <= 0;
            zp_reg      <= 0;
            valid_s1    <= 1'b0;
        end else begin
            valid_s1 <= in_valid;
            if (in_valid) begin
                product_reg <= data_in * mult_signed;
                shift_reg   <= shift;
                zp_reg      <= zero_point;
            end
        end
    end
    
    // ========================================================================
    // Stage 2: Shift, add zero point, and clamp
    // ========================================================================
    wire signed [PRODUCT_WIDTH-1:0] shifted;
    wire signed [PRODUCT_WIDTH-1:0] with_zp;
    
    // Arithmetic right shift (preserves sign)
    assign shifted = product_reg >>> shift_reg;
    
    // Add zero point
    assign with_zp = shifted + {{(PRODUCT_WIDTH-DATA_WIDTH){zp_reg[DATA_WIDTH-1]}}, zp_reg};
    
    // INT8 clamping
    localparam signed [DATA_WIDTH-1:0] INT8_MAX =  8'sd127;
    localparam signed [DATA_WIDTH-1:0] INT8_MIN = -8'sd128;
    
    always @(posedge clk) begin
        if (rst) begin
            data_out  <= 0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= valid_s1;
            if (valid_s1) begin
                if (with_zp > INT8_MAX)
                    data_out <= INT8_MAX;
                else if (with_zp < INT8_MIN)
                    data_out <= INT8_MIN;
                else
                    data_out <= with_zp[DATA_WIDTH-1:0];
            end
        end
    end

endmodule
