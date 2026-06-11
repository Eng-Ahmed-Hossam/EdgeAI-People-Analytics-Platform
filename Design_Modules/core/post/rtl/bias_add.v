/*
------------------------------------------------------------------------------
Module Name : bias_add
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Adds INT32 bias to convolution accumulator output.
    
    Functional Role:
    - First stage in post-convolution pipeline
    - Adds per-channel INT32 bias to INT32 accumulator result
    - Pass-through valid signal for pipeline flow
    
    Input/Output Contract:
    - acc_in: Signed INT32 accumulator value from PE array
    - bias_in: Signed INT32 bias value for current output channel
    - result: acc_in + bias_in (signed INT32)
    - in_valid/out_valid: Valid pipeline handshake
    
    Data Widths:
    - Input/Output: ACC_WIDTH (32-bit)
    - Bias: BIAS_WIDTH (32-bit)
    
    Timing:
    - 1 clock cycle latency (registered output)
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — bias addition

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module bias_add #(
    parameter ACC_WIDTH  = `ACC_WIDTH,
    parameter BIAS_WIDTH = `BIAS_WIDTH
)(
    input  wire                          clk,
    input  wire                          rst,
    
    input  wire signed [ACC_WIDTH-1:0]   acc_in,     // From PE accumulator
    input  wire signed [BIAS_WIDTH-1:0]  bias_in,    // Per-channel bias
    input  wire                          in_valid,
    
    output reg  signed [ACC_WIDTH-1:0]   result,     // acc + bias
    output reg                           out_valid
);

    // ========================================================================
    // Bias addition (registered)
    // ========================================================================
    // Sign-extend bias to ACC_WIDTH if needed, then add
    wire signed [ACC_WIDTH-1:0] bias_ext;
    
    generate
        if (BIAS_WIDTH < ACC_WIDTH) begin : bias_extend
            assign bias_ext = {{(ACC_WIDTH-BIAS_WIDTH){bias_in[BIAS_WIDTH-1]}}, bias_in};
        end else begin : bias_trunc
            assign bias_ext = bias_in[ACC_WIDTH-1:0];
        end
    endgenerate
    
    always @(posedge clk) begin
        if (rst) begin
            result    <= 0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                result <= acc_in + bias_ext;
            end
        end
    end

endmodule
