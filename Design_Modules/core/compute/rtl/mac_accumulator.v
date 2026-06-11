/*
------------------------------------------------------------------------------
Module Name : mac_accumulator
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Overflow-safe multiply-accumulate accumulator.
    
    Functional Role:
    - Provides a wider-than-32-bit internal accumulator (40-bit)
    - Detects overflow/saturation conditions
    - Supports clear, accumulate, and hold modes
    - Used alongside PE for extended accumulation safety
    
    Input/Output Contract:
    - data_in: Signed value to accumulate (ACC_WIDTH bits)
    - acc_out: Current accumulator value (truncated to ACC_WIDTH)
    - acc_full: Full internal accumulator (MAC_WIDTH bits)
    - overflow: Asserted if saturation occurred
    - mode: 00=hold, 01=accumulate, 10=clear, 11=load
    
    Data Widths:
    - Input: ACC_WIDTH (32-bit)
    - Internal: MAC_WIDTH (40-bit)
    - Output: ACC_WIDTH (32-bit, truncated)
    
    Timing:
    - Single-cycle accumulate
    - Registered output
    
    Reset:
    - Synchronous active-HIGH reset
    - Clears accumulator to zero

Dataflow    :
    Output-Stationary CNN accelerator pipeline — accumulation

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module mac_accumulator #(
    parameter ACC_WIDTH = `ACC_WIDTH,
    parameter MAC_WIDTH = `MAC_WIDTH
)(
    input  wire                        clk,
    input  wire                        rst,
    
    input  wire signed [ACC_WIDTH-1:0] data_in,    // Value to accumulate
    input  wire [1:0]                  mode,        // 00=hold, 01=acc, 10=clear, 11=load
    
    output wire signed [ACC_WIDTH-1:0] acc_out,     // Truncated output
    output reg  signed [MAC_WIDTH-1:0] acc_full,    // Full-precision output
    output reg                         overflow     // Saturation flag
);

    // ========================================================================
    // Mode encoding
    // ========================================================================
    localparam MODE_HOLD  = 2'b00;
    localparam MODE_ACC   = 2'b01;
    localparam MODE_CLEAR = 2'b10;
    localparam MODE_LOAD  = 2'b11;
    
    // ========================================================================
    // Extended accumulation
    // ========================================================================
    wire signed [MAC_WIDTH-1:0] data_ext;
    wire signed [MAC_WIDTH-1:0] sum;
    
    // Sign-extend input to MAC width
    assign data_ext = {{(MAC_WIDTH-ACC_WIDTH){data_in[ACC_WIDTH-1]}}, data_in};
    assign sum = acc_full + data_ext;
    
    // Saturation constants
    localparam signed [ACC_WIDTH-1:0] SAT_MAX = {1'b0, {(ACC_WIDTH-1){1'b1}}};  // +2^31 - 1
    localparam signed [ACC_WIDTH-1:0] SAT_MIN = {1'b1, {(ACC_WIDTH-1){1'b0}}};  // -2^31
    
    // Overflow detection: check if result exceeds 32-bit range
    wire overflow_detect;
    assign overflow_detect = (sum > {{(MAC_WIDTH-ACC_WIDTH){1'b0}}, SAT_MAX}) ||
                             (sum < {{(MAC_WIDTH-ACC_WIDTH){1'b1}}, SAT_MIN});
    
    // Truncated output with saturation
    assign acc_out = overflow ? (acc_full[MAC_WIDTH-1] ? SAT_MIN : SAT_MAX) :
                                acc_full[ACC_WIDTH-1:0];
    
    // ========================================================================
    // Accumulator register
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            acc_full <= {MAC_WIDTH{1'b0}};
            overflow <= 1'b0;
        end else begin
            case (mode)
                MODE_HOLD: begin
                    // Hold current value
                end
                
                MODE_ACC: begin
                    acc_full <= sum;
                    overflow <= overflow | overflow_detect;
                end
                
                MODE_CLEAR: begin
                    acc_full <= {MAC_WIDTH{1'b0}};
                    overflow <= 1'b0;
                end
                
                MODE_LOAD: begin
                    acc_full <= data_ext;
                    overflow <= 1'b0;
                end
                
                default: begin
                    // Hold
                end
            endcase
        end
    end

endmodule
