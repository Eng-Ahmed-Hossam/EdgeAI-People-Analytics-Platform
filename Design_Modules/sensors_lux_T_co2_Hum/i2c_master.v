// =============================================================================
// Module: i2c_master
// Description: Generic I2C Master Controller (100 kHz standard mode)
//              Handles START, STOP, byte write, byte read with ACK/NACK.
//              Used by both SCD30 and BH1750 sensor drivers.
//
// Clock Assumption: clk = 50 MHz  →  CLK_DIV = 250 gives 100 kHz SCL
// =============================================================================

module i2c_master #(
    parameter CLK_FREQ  = 50_000_000,   // System clock frequency (Hz)
    parameter I2C_FREQ  = 100_000       // I2C SCL frequency (Hz)
)(
    // ── System signals ──────────────────────────────────────────────────────
    input  wire        clk,             // System clock
    input  wire        rst_n,           // Active-low synchronous reset

    // ── Control / Data interface (from sensor drivers) ───────────────────
    input  wire        start,           // Pulse: begin transaction
    input  wire        stop,            // Pulse: send STOP condition
    input  wire        read,            // 1 = read byte, 0 = write byte
    input  wire  [7:0] data_in,         // Byte to transmit (write mode)
    input  wire        send_ack,        // 1 = master sends ACK after read

    output reg   [7:0] data_out,        // Byte received (read mode)
    output reg         done,            // Pulses high for 1 clk when done
    output reg         ack_error,       // 1 = slave did not ACK

    // ── I2C physical bus ────────────────────────────────────────────────
    output reg         scl,             // Serial clock (driven directly)
    inout  wire        sda              // Serial data (open-drain via tri-state)
);

    // ── Clock divider ──────────────────────────────────────────────────────
    localparam CLK_DIV = CLK_FREQ / (I2C_FREQ * 4); // Quarter-period count

    // ── FSM state encoding ─────────────────────────────────────────────────
    localparam [3:0]
        IDLE      = 4'd0,
        START_SDA = 4'd1,   // SDA goes low while SCL high  → START
        START_SCL = 4'd2,   // SCL goes low after START
        WR_DATA   = 4'd3,   // Clock out / in one byte (8 bits)
        WR_ACK    = 4'd4,   // Clock in slave ACK
        RD_ACK    = 4'd5,   // Clock out master ACK/NACK
        STOP_SCL  = 4'd6,   // SCL goes high before STOP
        STOP_SDA  = 4'd7;   // SDA goes high while SCL high → STOP

    reg [3:0]  state;
    reg [7:0]  clk_cnt;     // Quarter-period counter
    reg [2:0]  bit_cnt;     // Bit index (7 downto 0)
    reg [7:0]  shift_reg;   // Shift register for TX/RX
    reg        sda_out;     // Driven value on SDA
    reg        sda_en;      // 1 = master drives SDA

    // Open-drain SDA: drive 0 when enabled and sda_out=0, else hi-Z
    assign sda = (sda_en && !sda_out) ? 1'b0 : 1'bz;

    // ── Quarter-period tick ────────────────────────────────────────────────
    wire tick = (clk_cnt == CLK_DIV - 1);

    always @(posedge clk) begin
        if (!rst_n) begin
            clk_cnt <= 0;
        end else if (state != IDLE || start) begin
            clk_cnt <= tick ? 8'd0 : clk_cnt + 1'b1;
        end else begin
            clk_cnt <= 0;
        end
    end

    // ── Main FSM ───────────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (!rst_n) begin
            state     <= IDLE;
            scl       <= 1'b1;
            sda_out   <= 1'b1;
            sda_en    <= 1'b0;
            done      <= 1'b0;
            ack_error <= 1'b0;
            data_out  <= 8'd0;
            bit_cnt   <= 3'd7;
            shift_reg <= 8'd0;
        end else begin
            done <= 1'b0; // default: not done

            case (state)
                // ── IDLE: wait for start pulse ──────────────────────────
                IDLE: begin
                    scl     <= 1'b1;
                    sda_out <= 1'b1;
                    sda_en  <= 1'b0;
                    if (start) begin
                        sda_en  <= 1'b1;
                        state   <= START_SDA;
                    end
                end

                // ── START condition: pull SDA low while SCL high ─────────
                START_SDA: begin
                    if (tick) begin
                        sda_out <= 1'b0;   // SDA falls → START
                        state   <= START_SCL;
                    end
                end

                START_SCL: begin
                    if (tick) begin
                        scl       <= 1'b0; // SCL falls → begin data phase
                        shift_reg <= data_in;
                        bit_cnt   <= 3'd7;
                        state     <= WR_DATA;
                    end
                end

                // ── Data phase: 8 bits MSB first ─────────────────────────
                WR_DATA: begin
                    if (tick) begin
                        // Quarter-period sequencing using clk_cnt edges:
                        // Q0: set SDA, Q1: SCL high, Q2: sample (read), Q3: SCL low
                        // We re-use clk_cnt being 0 right after tick to track phase.
                        // Simplified: toggle SCL each tick; two ticks per bit
                        if (!scl) begin
                            // SCL is low: put next data bit on SDA
                            sda_en  <= 1'b1;
                            if (!read)
                                sda_out <= shift_reg[7];
                            else
                                sda_en <= 1'b0; // release for slave to drive
                            scl <= 1'b1;       // raise SCL
                        end else begin
                            // SCL is high: sample SDA (read) or shift (write)
                            if (read)
                                shift_reg <= {shift_reg[6:0], sda};
                            scl <= 1'b0;       // lower SCL
                            if (bit_cnt == 3'd0) begin
                                data_out <= read ? {shift_reg[6:0], sda} : data_out;
                                state    <= read ? RD_ACK : WR_ACK;
                            end else begin
                                shift_reg <= {shift_reg[6:0], 1'b0};
                                bit_cnt   <= bit_cnt - 1'b1;
                            end
                        end
                    end
                end

                // ── ACK from slave (after write) ─────────────────────────
                WR_ACK: begin
                    if (tick) begin
                        if (!scl) begin
                            sda_en <= 1'b0;    // release SDA for slave ACK
                            scl    <= 1'b1;
                        end else begin
                            ack_error <= sda;  // 0=ACK, 1=NACK
                            scl    <= 1'b0;
                            done   <= 1'b1;
                            state  <= stop ? STOP_SCL : IDLE;
                        end
                    end
                end

                // ── Master sends ACK/NACK after read ────────────────────
                RD_ACK: begin
                    if (tick) begin
                        if (!scl) begin
                            sda_en  <= 1'b1;
                            sda_out <= !send_ack; // 0=ACK, 1=NACK
                            scl     <= 1'b1;
                        end else begin
                            scl    <= 1'b0;
                            done   <= 1'b1;
                            state  <= stop ? STOP_SCL : IDLE;
                        end
                    end
                end

                // ── STOP condition: SCL high then SDA high ───────────────
                STOP_SCL: begin
                    if (tick) begin
                        sda_en  <= 1'b1;
                        sda_out <= 1'b0;   // ensure SDA low before SCL rises
                        scl     <= 1'b1;
                        state   <= STOP_SDA;
                    end
                end

                STOP_SDA: begin
                    if (tick) begin
                        sda_out <= 1'b1;   // SDA rises while SCL high → STOP
                        sda_en  <= 1'b0;
                        state   <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
