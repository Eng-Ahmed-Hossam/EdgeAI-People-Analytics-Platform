/*
------------------------------------------------------------------------------
Module Name : pe
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Processing Element for output-stationary CNN dataflow.
    
    Functional Role:
    - Performs signed 8x8 multiply-accumulate (MAC)
    - Accumulates partial sums locally (output-stationary)
    - Registered output for timing closure
    - Enable gating to reduce switching power
    
    Input/Output Contract:
    - ifmap_data: Signed INT8 activation input
    - weight_data: Signed INT8 weight input
    - acc_in: Previous accumulation value (INT32)
    - acc_out: Updated accumulation result (INT32)
    - en: Enable gate — when LOW, output holds previous value
    - acc_clear: Resets accumulator to zero
    
    Data Widths:
    - Multiply: 8 x 8 = 16-bit product
    - Accumulator: 32-bit (overflow checked by mac_accumulator)
    
    Timing:
    - Single-cycle multiply + add (combinational product, registered sum)
    - 1 clock cycle latency from inputs to acc_out
    
    Reset:
    - Synchronous active-HIGH reset
    - Clears accumulator to zero

Dataflow    :
    Output-Stationary CNN accelerator pipeline — PE element

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module pe #(
    parameter DATA_WIDTH = `DATA_WIDTH,
    parameter ACC_WIDTH  = `ACC_WIDTH
)(
    input  wire                        clk,
    input  wire                        rst,
    input  wire                        en,          // Enable gate
    input  wire                        acc_clear,   // Clear accumulator
    
    input  wire signed [DATA_WIDTH-1:0] ifmap_data,  // Activation (INT8)
    input  wire signed [DATA_WIDTH-1:0] weight_data, // Weight (INT8)
    
    output reg  signed [ACC_WIDTH-1:0]  acc_out      // Accumulated result
);

    // ========================================================================
    // Internal signals
    // ========================================================================
    wire signed [2*DATA_WIDTH-1:0] product;
    wire signed [ACC_WIDTH-1:0]    sum;
    
    // Signed multiplication: 8x8 -> 16-bit
    assign product = ifmap_data * weight_data;
    
    // Addition: sign-extend product to ACC_WIDTH and add
    assign sum = acc_out + {{(ACC_WIDTH-2*DATA_WIDTH){product[2*DATA_WIDTH-1]}}, product};
    
    // ========================================================================
    // Registered accumulator
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            acc_out <= {ACC_WIDTH{1'b0}};
        end else if (acc_clear) begin
            acc_out <= {ACC_WIDTH{1'b0}};
        end else if (en) begin
            acc_out <= sum;
        end
        // else: hold previous value (enable gating)
    end

endmodule
