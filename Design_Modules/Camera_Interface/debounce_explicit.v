/*
 * debounce_explicit.v
 *
 * Push-button Debouncer (Explicit FSM + Timer)
 *
 * This module removes mechanical switch bounce by using a finite state
 * machine and a configurable counter. When a press is validated, db_level
 * goes high, and a 1-cycle db_tick pulse is generated.
 *
 * Parameters:
 * - N : counter width used for debounce interval (50 MHz → ~42 ms for N=21)
 *
 * Inputs:
 * - clk   : system clock
 * - rst_n : active-low reset
 * - sw    : raw mechanical switch input
 *
 * Outputs:
 * - db_level : stable (debounced) switch level
 * - db_tick  : 1-clock pulse on valid rising edge
 */

`timescale 1ns/1ps

module debounce_explicit (
    input  wire clk,
    input  wire rst_n,
    input  wire sw,
    output reg  db_level,     // stable output
    output reg  db_tick       // 1-cycle pulse on press
);

    // -------------------------------------------------------------------------
    // State Machine Encoding
    // IDLE   : waiting for a press
    // DELAY0 : sw=1 must remain stable long enough → debounce press
    // ONE    : debounced high state
    // DELAY1 : sw=0 must remain stable long enough → debounce release
    // -------------------------------------------------------------------------
    localparam [1:0]
        IDLE   = 2'b00,
        DELAY0 = 2'b01,
        ONE    = 2'b10,
        DELAY1 = 2'b11;

    localparam N = 21;        // 42 ms debounce window at 50 MHz

    reg [1:0]     state_reg, state_next;
    reg [N-1:0]   timer_reg, timer_next;

    wire timer_tick = (timer_reg == {N{1'b1}});

    // control signals for timer
    reg timer_zero;
    reg timer_inc;

    // -------------------------------------------------------------------------
    // Sequential Logic (State + Timer + Outputs)
    // -------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_reg <= IDLE;
            timer_reg <= 0;
            db_level  <= 0;
            db_tick   <= 0;
        end else begin
            state_reg <= state_next;
            timer_reg <= timer_next;

            // debounced stable level: asserted when FSM is in a "pressed" region
            db_level  <= (state_next == ONE || state_next == DELAY1);

            // 1-clock pulse on valid press detection
            db_tick   <= (state_reg == DELAY0 && state_next == ONE);
        end
    end

    // -------------------------------------------------------------------------
    // Combinational FSM: Next State Logic + Timer Control
    // -------------------------------------------------------------------------
    always @(*) begin
        // defaults
        state_next = state_reg;
        timer_zero = 0;
        timer_inc  = 0;

        case (state_reg)

            // -------------------------------------------------
            // IDLE → wait for raw switch = 1
            // -------------------------------------------------
            IDLE: begin
                if (sw) begin
                    timer_zero = 1;     // start debounce timer
                    state_next = DELAY0;
                end
            end

            // -------------------------------------------------
            // DELAY0 → confirm press is stable
            // -------------------------------------------------
            DELAY0: begin
                if (sw) begin
                    timer_inc = 1;       // keep counting
                    if (timer_tick)
                        state_next = ONE; // confirmed press
                end else begin
                    state_next = IDLE;    // bounce → return
                end
            end

            // -------------------------------------------------
            // ONE → debounced HIGH state
            // -------------------------------------------------
            ONE: begin
                if (!sw) begin
                    timer_zero = 1;        // start release debounce timer
                    state_next = DELAY1;
                end
            end

            // -------------------------------------------------
            // DELAY1 → confirm release is stable
            // -------------------------------------------------
            DELAY1: begin
                if (!sw) begin
                    timer_inc = 1;
                    if (timer_tick)
                        state_next = IDLE; // confirmed release
                end else begin
                    state_next = ONE;      // bounce back to HIGH
                end
            end

        endcase
    end

    // -------------------------------------------------------------------------
    // Timer Combinational Logic
    // -------------------------------------------------------------------------
    always @(*) begin
        timer_next = timer_reg;

        if (timer_zero)
            timer_next = 0;
        else if (timer_inc)
            timer_next = timer_reg + 1'b1;
    end

endmodule
