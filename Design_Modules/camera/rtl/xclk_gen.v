/*
------------------------------------------------------------------------------
Module Name : xclk_gen
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Generates XCLK master clock for OV7670 camera sensor.
    
    Functional Role:
    - Divides system clock to produce XCLK (typically 24-25 MHz)
    - OV7670 uses XCLK as its internal master clock
    - PCLK is derived from XCLK inside the sensor
    
    Input/Output Contract:
    - sys_clk: System clock input (e.g., 100 MHz)
    - xclk: Output clock at sys_clk / (2 * DIV_FACTOR)
    
    Data Widths:
    - DIV_FACTOR parameter controls division ratio
    
    Timing:
    - For 100 MHz sys_clk and DIV_FACTOR=2: XCLK = 25 MHz
    - 50% duty cycle output
    
    Reset:
    - Synchronous active-HIGH reset
    - Drives XCLK LOW on reset

Dataflow    :
    Output-Stationary CNN accelerator pipeline — camera clock generation

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module xclk_gen #(
    parameter DIV_FACTOR = `XCLK_DIV / 2   // Half-period counter value
)(
    input  wire clk,       // System clock
    input  wire rst,       // Synchronous reset (active HIGH)
    output reg  xclk       // Generated XCLK for OV7670
);

    // ========================================================================
    // Clock divider
    // ========================================================================
    reg [$clog2(DIV_FACTOR+1)-1:0] cnt;
    
    always @(posedge clk) begin
        if (rst) begin
            cnt  <= 0;
            xclk <= 1'b0;
        end else begin
            if (cnt == DIV_FACTOR - 1) begin
                cnt  <= 0;
                xclk <= ~xclk;
            end else begin
                cnt <= cnt + 1;
            end
        end
    end

endmodule
