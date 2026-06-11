/*
------------------------------------------------------------------------------
Module Name : activation_unit
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Activation function unit supporting ReLU and LeakyReLU.
    
    Functional Role:
    - Applies nonlinear activation to requantized INT8 value
    - ReLU: out = max(0, in)
    - LeakyReLU: out = (in < 0) ? (in >>> LEAK_SHIFT) : in
    - Bypass mode for detection head layer
    
    Input/Output Contract:
    - data_in: Signed INT8 input
    - data_out: Signed INT8 activated output
    - mode: Activation mode (ACT_RELU, ACT_LEAKY_RELU, ACT_NONE)
    - in_valid/out_valid: Pipeline valid handshake
    
    Data Widths:
    - Input/Output: DATA_WIDTH (8-bit)
    
    Timing:
    - 1 clock cycle latency (registered output)
    
    Reset:
    - Synchronous active-HIGH reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — activation

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module activation_unit #(
    parameter DATA_WIDTH  = `DATA_WIDTH,
    parameter LEAK_SHIFT  = `LEAK_SHIFT
)(
    input  wire                         clk,
    input  wire                         rst,
    
    input  wire signed [DATA_WIDTH-1:0] data_in,
    input  wire                         in_valid,
    input  wire [1:0]                   mode,        // ACT_RELU / ACT_LEAKY_RELU / ACT_NONE
    
    output reg  signed [DATA_WIDTH-1:0] data_out,
    output reg                          out_valid
);

    // ========================================================================
    // Activation logic
    // ========================================================================
    wire is_negative;
    wire signed [DATA_WIDTH-1:0] relu_result;
    wire signed [DATA_WIDTH-1:0] leaky_result;
    
    assign is_negative = data_in[DATA_WIDTH-1];
    
    // ReLU: clamp negatives to zero
    assign relu_result = is_negative ? {DATA_WIDTH{1'b0}} : data_in;
    
    // LeakyReLU: arithmetic right shift for negative values
    // Approximates slope = 1/2^LEAK_SHIFT (~0.125 for LEAK_SHIFT=3)
    assign leaky_result = is_negative ? (data_in >>> LEAK_SHIFT) : data_in;
    
    // ========================================================================
    // Registered output with mode selection
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            data_out  <= 0;
            out_valid <= 1'b0;
        end else begin
            out_valid <= in_valid;
            if (in_valid) begin
                case (mode)
                    `ACT_RELU:       data_out <= relu_result;
                    `ACT_LEAKY_RELU: data_out <= leaky_result;
                    `ACT_NONE:       data_out <= data_in;     // Passthrough
                    default:         data_out <= data_in;
                endcase
            end
        end
    end

endmodule
