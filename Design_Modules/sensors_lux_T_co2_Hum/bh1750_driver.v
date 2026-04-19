// =============================================================================
// Module: bh1750_driver
// Description: I2C driver for the ROHM BH1750 Ambient Light Sensor.
//              Implements One-Time High-Resolution Mode (0x20).
//              Reads a 16-bit raw light value and converts to lux.
//
// I2C Address: 0x23 (ADDR pin LOW)  or  0x5C (ADDR pin HIGH)
// Commands:
//   0x01 – Power On
//   0x07 – Reset data register
//   0x20 – One-Time H-Resolution Mode (1 lx resolution, ~120 ms)
//   0x10 – Continuous H-Resolution Mode
//
// Lux conversion: lux = raw_count / 1.2
//   For FPGA simplicity: lux_x10 = (raw_count * 10) / 12  (integer arithmetic)
// =============================================================================

module bh1750_driver (
    // ── System ──────────────────────────────────────────────────────────────
    input  wire        clk,         // 50 MHz system clock
    input  wire        rst_n,       // Active-low reset

    // ── User Control ────────────────────────────────────────────────────────
    input  wire        enable,      // Start continuous measurement
    input  wire        addr_sel,    // 0 = 0x23, 1 = 0x5C

    // ── Outputs ──────────────────────────────────────────────────────────────
    output reg         data_valid,  // Pulses 1 clk when lux_x10 is updated
    output reg [15:0]  lux_x10,     // Lux value × 10 (e.g. 1234 = 123.4 lux)
    output reg [15:0]  raw_count,   // Raw 16-bit sensor reading

    // ── I2C bus ─────────────────────────────────────────────────────────────
    output wire        sda,
    output wire        scl
);

    // ── I2C Address selection ─────────────────────────────────────────────
    wire [6:0] BH1750_ADDR = addr_sel ? 7'h5C : 7'h23;

    // ── Commands ─────────────────────────────────────────────────────────
    localparam [7:0]
        CMD_POWER_ON = 8'h01,
        CMD_RESET    = 8'h07,
        CMD_ONE_TIME = 8'h20;   // One-Time H-Resolution Mode

    // ── Measurement time: ~120 ms = 6_000_000 cycles @ 50 MHz ────────────
    localparam [31:0] MEAS_WAIT = 32'd6_000_000;

    reg [31:0] wait_cnt;
    wire       wait_done = (wait_cnt == 0);

    // ── I2C master interface ──────────────────────────────────────────────
    reg        m_start, m_stop, m_read, m_send_ack;
    reg  [7:0] m_data_in;
    wire [7:0] m_data_out;
    wire       m_done, m_ack_error;

    i2c_master #(
        .CLK_FREQ(50_000_000),
        .I2C_FREQ(100_000)
    ) u_i2c (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (m_start),
        .stop     (m_stop),
        .read     (m_read),
        .data_in  (m_data_in),
        .send_ack (m_send_ack),
        .data_out (m_data_out),
        .done     (m_done),
        .ack_error(m_ack_error),
        .scl      (scl),
        .sda      (sda)
    );

    // ── FSM States ────────────────────────────────────────────────────────
    localparam [3:0]
        S_IDLE       = 4'd0,
        S_PWR_ADDR   = 4'd1,   // START + write address
        S_PWR_CMD    = 4'd2,   // Send POWER_ON command
        S_PWR_STOP   = 4'd3,
        S_TRIG_ADDR  = 4'd4,   // START + write address (trigger measurement)
        S_TRIG_CMD   = 4'd5,   // Send ONE_TIME command
        S_TRIG_STOP  = 4'd6,
        S_WAIT_MEAS  = 4'd7,   // Wait 120 ms
        S_RD_ADDR    = 4'd8,   // START + read address
        S_RD_MSB     = 4'd9,   // Read high byte
        S_RD_LSB     = 4'd10,  // Read low byte (NACK)
        S_COMPUTE    = 4'd11;  // Compute lux, latch outputs

    reg [3:0] state;
    reg [7:0] msb_buf;      // Temporary store for high byte

    // ── Wait counter ─────────────────────────────────────────────────────
    always @(posedge clk) begin
        if (!rst_n)
            wait_cnt <= MEAS_WAIT;
        else if (state == S_WAIT_MEAS)
            wait_cnt <= wait_done ? 0 : wait_cnt - 1;
        else
            wait_cnt <= MEAS_WAIT;
    end

    // ── Enable rising-edge detector ───────────────────────────────────────
    // enable_prev is intentionally NOT cleared on reset. This preserves the
    // actual value of enable across reset so that if enable=1 both before and
    // after reset, enable_prev=1 after reset too — meaning no rising edge is
    // seen and the FSM correctly stays in S_IDLE until a genuine 0→1 pulse.
    reg enable_prev;

    // ── FSM ───────────────────────────────────────────────────────────────
    always @(posedge clk) begin
        enable_prev <= enable;   // always runs — never gated by reset

        if (!rst_n) begin
            state      <= S_IDLE;
            m_start    <= 0; m_stop <= 0; m_read <= 0; m_send_ack <= 0;
            m_data_in  <= 0;
            data_valid <= 0;
            lux_x10    <= 0;
            raw_count  <= 0;
            msb_buf    <= 0;
        end else begin
            m_start <= 0; m_stop <= 0;
            data_valid <= 0;

            case (state)
                S_IDLE: begin
                    if (enable && !enable_prev) state <= S_PWR_ADDR;
                end

                // ── Power On sequence ──────────────────────────────────
                S_PWR_ADDR: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {BH1750_ADDR, 1'b0};
                    if (m_done) state <= S_PWR_CMD;
                end

                S_PWR_CMD: begin
                    m_data_in <= CMD_POWER_ON;
                    m_stop    <= 1;
                    if (m_done) state <= S_TRIG_ADDR;
                end

                // ── Trigger One-Time H-Res Measurement ────────────────
                S_TRIG_ADDR: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {BH1750_ADDR, 1'b0};
                    if (m_done) state <= S_TRIG_CMD;
                end

                S_TRIG_CMD: begin
                    m_data_in <= CMD_ONE_TIME;
                    m_stop    <= 1;
                    if (m_done) state <= S_WAIT_MEAS;
                end

                // ── Wait 120 ms for measurement ────────────────────────
                S_WAIT_MEAS: begin
                    if (wait_done) state <= S_RD_ADDR;
                end

                // ── Read 2 bytes from sensor ───────────────────────────
                S_RD_ADDR: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {BH1750_ADDR, 1'b1}; // read address
                    if (m_done) state <= S_RD_MSB;
                end

                S_RD_MSB: begin
                    m_read     <= 1;
                    m_send_ack <= 1;   // ACK → more bytes coming
                    if (m_done) begin
                        msb_buf <= m_data_out;
                        state   <= S_RD_LSB;
                    end
                end

                S_RD_LSB: begin
                    m_read     <= 1;
                    m_send_ack <= 0;   // NACK → last byte
                    m_stop     <= 1;
                    if (m_done) begin
                        raw_count <= {msb_buf, m_data_out};
                        state     <= S_COMPUTE;
                    end
                end

                // ── Compute lux_x10 = raw * 10 / 12 ──────────────────
                S_COMPUTE: begin
                    // Multiply raw by 10, divide by 12
                    // (raw * 10) fits in 20 bits since raw is 16 bits
                    lux_x10    <= ({4'b0, raw_count} * 10) / 12;
                    data_valid <= 1;

                    // Re-trigger: send another One-Time measurement
                    state <= enable ? S_TRIG_ADDR : S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule