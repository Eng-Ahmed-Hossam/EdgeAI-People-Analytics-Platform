/*
------------------------------------------------------------------------------
Module Name : sccb_ctrl
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    SCCB (Serial Camera Control Bus) controller for OV7670 configuration.
    
    Functional Role:
    - Implements SCCB write-only protocol (I2C-like)
    - Sends 3-phase writes: Device ID + Sub-address + Data
    - Used to configure OV7670 registers at startup
    
    Input/Output Contract:
    - start pulse triggers a 3-phase SCCB write
    - addr[7:0] = register sub-address
    - data[7:0] = register data to write  
    - done pulses HIGH when transaction completes
    - busy stays HIGH during transaction
    
    Data Widths:
    - 8-bit sub-address, 8-bit data
    - Device ID hardcoded as parameter (default OV7670 = 0x42)
    
    Timing:
    - System clock divided by CLK_DIV for SCCB timing
    - ~400 KHz SCCB clock from 100 MHz system clock
    
    Reset:
    - Synchronous active-HIGH reset
    - Tri-states SDA, drives SCL HIGH

Dataflow    :
    Output-Stationary CNN accelerator pipeline — camera configuration

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module sccb_ctrl #(
    parameter CLK_DIV    = `SCCB_CLK_DIV,
    parameter DEVICE_ID  = `OV7670_ADDR
)(
    input  wire       clk,
    input  wire       rst,
    
    // Control interface
    input  wire       start,        // Start transaction pulse
    input  wire [7:0] addr,         // Register sub-address
    input  wire [7:0] data_in,      // Register data to write
    output reg        done,         // Transaction complete pulse
    output reg        busy,         // Transaction in progress
    
    // SCCB physical interface
    output reg        sccb_scl,     // Serial clock
    inout  wire       sccb_sda      // Serial data (directly directly, directly directly)
);

    // ========================================================================
    // Clock divider for SCCB timing
    // ========================================================================
    localparam CLK_DIV_HALF = CLK_DIV / 2;
    
    reg [$clog2(CLK_DIV)-1:0] clk_cnt;
    reg                        clk_tick;   // Tick at SCCB rate
    
    always @(posedge clk) begin
        if (rst || !busy) begin
            clk_cnt  <= 0;
            clk_tick <= 1'b0;
        end else begin
            clk_tick <= 1'b0;
            if (clk_cnt == CLK_DIV - 1) begin
                clk_cnt  <= 0;
                clk_tick <= 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end
    end
    
    // ========================================================================
    // SCCB state machine
    // ========================================================================
    // 3-phase write: [START][ID byte][ACK][ADDR byte][ACK][DATA byte][ACK][STOP]
    // Total bit states: START(1) + 8 bits ID + ACK(1) + 8 bits ADDR + ACK(1) 
    //                   + 8 bits DATA + ACK(1) + STOP(1) = 30 half-cycles
    
    localparam PHASE_BITS = 6;
    localparam TOTAL_PHASES = 33;  // Total SCL half-cycle phases
    
    reg [PHASE_BITS-1:0] phase_cnt;
    reg                   sda_out;
    reg                   sda_oe;     // Output enable for SDA
    reg [7:0]             id_reg;
    reg [7:0]             addr_reg;
    reg [7:0]             data_reg;
    reg                   scl_phase;  // 0=low, 1=high within each bit
    
    // SDA tri-state: output when oe=1, high-Z when oe=0
    assign sccb_sda = sda_oe ? sda_out : 1'bz;
    
    // Bit index within each byte (MSB first, 7 downto 0)
    // Phase layout:
    // 0: START condition
    // 1-16: ID byte (8 bits x 2 phases each)
    // 17-18: ACK1
    // 19-34: ADDR byte (8 bits x 2 phases)  -> but we compress
    // We simplify: count bit-level, toggle SCL
    
    reg [5:0] bit_cnt;     // Bit counter within transaction
    reg [23:0] shift_reg;  // {ID[7:0], ADDR[7:0], DATA[7:0]}
    
    localparam S_IDLE    = 3'd0;
    localparam S_START   = 3'd1;
    localparam S_DATA    = 3'd2;
    localparam S_ACK     = 3'd3;
    localparam S_STOP    = 3'd4;
    localparam S_DONE    = 3'd5;
    
    reg [2:0] state;
    reg [1:0] byte_idx;    // Which byte (0=ID, 1=ADDR, 2=DATA)
    reg [2:0] bit_pos;     // Bit position within byte (7 downto 0)
    
    always @(posedge clk) begin
        if (rst) begin
            state     <= S_IDLE;
            sccb_scl  <= 1'b1;
            sda_out   <= 1'b1;
            sda_oe    <= 1'b1;
            busy      <= 1'b0;
            done      <= 1'b0;
            byte_idx  <= 2'd0;
            bit_pos   <= 3'd7;
            shift_reg <= 24'd0;
            scl_phase <= 1'b0;
        end else begin
            done <= 1'b0;
            
            case (state)
                S_IDLE: begin
                    sccb_scl <= 1'b1;
                    sda_out  <= 1'b1;
                    sda_oe   <= 1'b1;
                    if (start) begin
                        busy      <= 1'b1;
                        shift_reg <= {DEVICE_ID, addr, data_in};
                        byte_idx  <= 2'd0;
                        bit_pos   <= 3'd7;
                        state     <= S_START;
                    end
                end
                
                S_START: begin
                    if (clk_tick) begin
                        // START condition: SDA goes LOW while SCL is HIGH
                        sda_out <= 1'b0;
                        state   <= S_DATA;
                        sccb_scl <= 1'b0;  // Pull SCL low for first data bit
                    end
                end
                
                S_DATA: begin
                    if (clk_tick) begin
                        if (!scl_phase) begin
                            // SCL LOW phase: set SDA bit
                            sda_oe  <= 1'b1;
                            case (byte_idx)
                                2'd0: sda_out <= shift_reg[23 - (3'd7 - bit_pos)];
                                2'd1: sda_out <= shift_reg[15 - (3'd7 - bit_pos)];
                                2'd2: sda_out <= shift_reg[7  - (3'd7 - bit_pos)];
                                default: sda_out <= 1'b1;
                            endcase
                            sccb_scl  <= 1'b0;
                            scl_phase <= 1'b1;
                        end else begin
                            // SCL HIGH phase: clock the bit
                            sccb_scl  <= 1'b1;
                            scl_phase <= 1'b0;
                            
                            if (bit_pos == 3'd0) begin
                                // Byte complete, go to ACK
                                bit_pos <= 3'd7;
                                state   <= S_ACK;
                            end else begin
                                bit_pos <= bit_pos - 3'd1;
                            end
                        end
                    end
                end
                
                S_ACK: begin
                    if (clk_tick) begin
                        if (!scl_phase) begin
                            // Release SDA for ACK (don't care — SCCB)
                            sda_oe    <= 1'b0;
                            sccb_scl  <= 1'b0;
                            scl_phase <= 1'b1;
                        end else begin
                            sccb_scl  <= 1'b1;
                            scl_phase <= 1'b0;
                            sda_oe    <= 1'b1;
                            
                            if (byte_idx == 2'd2) begin
                                // All 3 bytes sent
                                state <= S_STOP;
                            end else begin
                                byte_idx <= byte_idx + 2'd1;
                                state    <= S_DATA;
                            end
                        end
                    end
                end
                
                S_STOP: begin
                    if (clk_tick) begin
                        if (!scl_phase) begin
                            sda_out   <= 1'b0;
                            sccb_scl  <= 1'b0;
                            scl_phase <= 1'b1;
                        end else begin
                            // STOP condition: SDA goes HIGH while SCL is HIGH
                            sccb_scl  <= 1'b1;
                            sda_out   <= 1'b1;
                            scl_phase <= 1'b0;
                            state     <= S_DONE;
                        end
                    end
                end
                
                S_DONE: begin
                    done  <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
