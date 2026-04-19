// =============================================================================
// Module: scd30_driver
// Description: I2C driver for the Sensirion SCD30 CO2/Temperature/Humidity
//              sensor. Implements the "Trigger Continuous Measurement" command
//              and periodic "Read Measurement" (6 x 16-bit words + CRC).
//
// I2C Address: 0x61  (7-bit)
// Key Commands:
//   0x0010 – Trigger Continuous Measurement (with optional pressure arg)
//   0x0202 – Start/Stop/Read Measurement Interval
//   0x0300 – Read Measurement (18 bytes: 6 words, each word has MSB,LSB,CRC)
//   0x0202 – Set Measurement Interval
//
// Data format returned (18 bytes):
//   Bytes  0- 2: CO2 float MSW  + CRC
//   Bytes  3- 5: CO2 float LSW  + CRC
//   Bytes  6- 8: Temp float MSW + CRC
//   Bytes  9-11: Temp float LSW + CRC
//   Bytes 12-14: Hum float MSW  + CRC
//   Bytes 15-17: Hum float LSW  + CRC
//
// NOTE: IEEE 754 float decode is omitted for FPGA simplicity; raw 32-bit
//       words are exported. Post-processing can happen in a soft-core CPU.
// =============================================================================

module scd30_driver (
    // ── System ──────────────────────────────────────────────────────────────
    input  wire        clk,         // 50 MHz system clock
    input  wire        rst_n,       // Active-low reset

    // ── User Control ────────────────────────────────────────────────────────
    input  wire        enable,      // High: start continuous measurement mode
    input  wire [15:0] interval_ms, // Measurement interval (2–1800 s, in ms)
    output reg         data_valid,  // Pulses 1 clk when new data is ready

    // ── Measurement outputs (raw 32-bit IEEE 754 words) ──────────────────
    output reg [31:0]  co2_raw,     // CO2 concentration (ppm, IEEE 754)
    output reg [31:0]  temp_raw,    // Temperature (°C,   IEEE 754)
    output reg [31:0]  hum_raw,     // Relative humidity (%, IEEE 754)
    output reg         crc_error,   // 1 = CRC mismatch detected

    // ── I2C bus ─────────────────────────────────────────────────────────────
    output wire        sda,
    output wire        scl
);

    // ── I2C Address ───────────────────────────────────────────────────────
    localparam [6:0] SCD30_ADDR = 7'h61;

    // ── Commands (16-bit, sent MSB first as two bytes) ────────────────────
    localparam [15:0] CMD_TRIG    = 16'h0010; // Trigger continuous measurement
    localparam [15:0] CMD_READ    = 16'h0300; // Read measurement

    // ── Timing: interval counter at 50 MHz ───────────────────────────────
    // interval_ms expressed in milliseconds; 1 ms = 50_000 clock cycles
    localparam [31:0] CLK_PER_MS = 32'd50_000;

    reg [31:0] interval_cnt;
    wire       time_to_read;

    // ── I2C master interface wires ────────────────────────────────────────
    reg        m_start, m_stop, m_read, m_send_ack;
    reg  [7:0] m_data_in;
    wire [7:0] m_data_out;
    wire       m_done, m_ack_error;

    // ── I2C Master instantiation ─────────────────────────────────────────
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

    // ── FSM states ────────────────────────────────────────────────────────
    localparam [4:0]
        S_IDLE        = 5'd0,
        // --- Configure continuous measurement ---
        S_CMD_START   = 5'd1,   // Send START + address (W)
        S_CMD_MSB     = 5'd2,   // Send command MSB
        S_CMD_LSB     = 5'd3,   // Send command LSB
        S_ARG_MSB     = 5'd4,   // Send argument MSB (pressure = 0)
        S_ARG_LSB     = 5'd5,   // Send argument LSB
        S_ARG_CRC     = 5'd6,   // Send argument CRC
        S_CMD_STOP    = 5'd7,   // Send STOP
        // --- Wait for interval ---
        S_WAIT        = 5'd8,
        // --- Read measurement: write phase ---
        S_RD_ADDR_W   = 5'd9,   // START + address (W) + READ cmd
        S_RD_CMD_MSB  = 5'd10,
        S_RD_CMD_LSB  = 5'd11,
        S_RD_STOP     = 5'd12,
        // --- Read measurement: read phase ---
        S_RD_ADDR_R   = 5'd13,  // Repeated START + address (R)
        S_RD_BYTES    = 5'd14,  // Clock in 18 bytes
        S_RD_DONE     = 5'd15;

    reg [4:0]  state;
    reg [4:0]  byte_idx;         // 0..17 during read phase
    reg [7:0]  rx_buf [0:17];    // Raw received bytes
    reg        configured;       // 1 = continuous mode started

    // ── CRC-8 (polynomial 0x31, init 0xFF) for SCD30 ─────────────────────
    function automatic [7:0] crc8;
        input [7:0] b0, b1;
        integer i;
        reg [7:0] crc;
        begin
            crc = 8'hFF;
            // byte 0
            crc = crc ^ b0;
            for (i = 0; i < 8; i = i + 1)
                crc = crc[7] ? (crc << 1) ^ 8'h31 : (crc << 1);
            // byte 1
            crc = crc ^ b1;
            for (i = 0; i < 8; i = i + 1)
                crc = crc[7] ? (crc << 1) ^ 8'h31 : (crc << 1);
            crc8 = crc;
        end
    endfunction

    // ── Interval timer ────────────────────────────────────────────────────
    assign time_to_read = (interval_cnt == 0);

    always @(posedge clk) begin
        if (!rst_n || state == S_IDLE) begin
            interval_cnt <= (interval_ms > 0) ? CLK_PER_MS * interval_ms : CLK_PER_MS * 2000;
        end else if (state == S_WAIT) begin
            interval_cnt <= time_to_read ? 0 : interval_cnt - 1;
        end
    end

    // ── Enable rising-edge detector ───────────────────────────────────────
    // The FSM leaves IDLE only on a genuine 0→1 rising edge of enable.
    // enable_prev is intentionally NOT cleared by rst_n. This preserves the
    // pre-reset value of enable so that if enable=1 during reset, the post-
    // reset cycle sees enable_prev=1 as well — which is NOT a rising edge —
    // and the FSM correctly stays in S_IDLE. Only a genuine software pulse
    // (enable goes low, then high again) causes the FSM to start.
    reg enable_prev;

    // ── FSM ───────────────────────────────────────────────────────────────
    integer k;

    always @(posedge clk) begin
        // enable_prev always tracks the previous cycle value — never reset
        enable_prev <= enable;

        if (!rst_n) begin
            state       <= S_IDLE;
            m_start     <= 0; m_stop <= 0; m_read <= 0; m_send_ack <= 0;
            m_data_in   <= 0;
            data_valid  <= 0;
            crc_error   <= 0;
            configured  <= 0;
            byte_idx    <= 0;
            co2_raw     <= 0; temp_raw <= 0; hum_raw <= 0;
            for (k = 0; k < 18; k = k + 1) rx_buf[k] <= 0;
        end else begin
            // Default pulses
            m_start <= 0; m_stop <= 0;
            data_valid <= 0;

            case (state)
                S_IDLE: begin
                    // Depart only on a genuine rising edge of enable
                    if (enable && !enable_prev) state <= S_CMD_START;
                end

                // ── Trigger Continuous Measurement ──────────────────────
                S_CMD_START: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {SCD30_ADDR, 1'b0}; // write address
                    if (m_done) state <= S_CMD_MSB;
                end

                S_CMD_MSB: begin
                    m_data_in <= CMD_TRIG[15:8];
                    if (m_done) state <= S_CMD_LSB;
                end

                S_CMD_LSB: begin
                    m_data_in <= CMD_TRIG[7:0];
                    if (m_done) state <= S_ARG_MSB;
                end

                S_ARG_MSB: begin   // Ambient pressure argument = 0 (disable)
                    m_data_in <= 8'h00;
                    if (m_done) state <= S_ARG_LSB;
                end

                S_ARG_LSB: begin
                    m_data_in <= 8'h00;
                    if (m_done) state <= S_ARG_CRC;
                end

                S_ARG_CRC: begin
                    m_data_in <= crc8(8'h00, 8'h00); // CRC of 0x0000
                    m_stop    <= 1;
                    if (m_done) begin
                        configured <= 1;
                        state      <= S_WAIT;
                    end
                end

                S_CMD_STOP: begin
                    m_stop <= 1;
                    if (m_done) state <= S_WAIT;
                end

                // ── Wait for interval ────────────────────────────────────
                S_WAIT: begin
                    if (time_to_read) state <= S_RD_ADDR_W;
                end

                // ── Write READ command to sensor ─────────────────────────
                S_RD_ADDR_W: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {SCD30_ADDR, 1'b0};
                    if (m_done) state <= S_RD_CMD_MSB;
                end

                S_RD_CMD_MSB: begin
                    m_data_in <= CMD_READ[15:8];
                    if (m_done) state <= S_RD_CMD_LSB;
                end

                S_RD_CMD_LSB: begin
                    m_data_in <= CMD_READ[7:0];
                    m_stop    <= 1;
                    if (m_done) state <= S_RD_ADDR_R;
                end

                // ── Repeated START + address (read) ─────────────────────
                S_RD_ADDR_R: begin
                    m_start   <= 1;
                    m_read    <= 0;
                    m_data_in <= {SCD30_ADDR, 1'b1}; // read address
                    byte_idx  <= 0;
                    if (m_done) state <= S_RD_BYTES;
                end

                // ── Clock in 18 bytes ────────────────────────────────────
                S_RD_BYTES: begin
                    m_read     <= 1;
                    m_send_ack <= (byte_idx < 17); // NACK on last byte
                    m_stop     <= (byte_idx == 17);
                    if (m_done) begin
                        rx_buf[byte_idx] <= m_data_out;
                        if (byte_idx == 17)
                            state <= S_RD_DONE;
                        else
                            byte_idx <= byte_idx + 1;
                    end
                end

                // ── Validate CRC and latch outputs ───────────────────────
                S_RD_DONE: begin
                    // Check CRC for each of the 6 word pairs
                    crc_error <=
                        (crc8(rx_buf[0],  rx_buf[1])  != rx_buf[2])  ||
                        (crc8(rx_buf[3],  rx_buf[4])  != rx_buf[5])  ||
                        (crc8(rx_buf[6],  rx_buf[7])  != rx_buf[8])  ||
                        (crc8(rx_buf[9],  rx_buf[10]) != rx_buf[11]) ||
                        (crc8(rx_buf[12], rx_buf[13]) != rx_buf[14]) ||
                        (crc8(rx_buf[15], rx_buf[16]) != rx_buf[17]);

                    // Assemble 32-bit floats
                    co2_raw  <= {rx_buf[0],  rx_buf[1],  rx_buf[3],  rx_buf[4]};
                    temp_raw <= {rx_buf[6],  rx_buf[7],  rx_buf[9],  rx_buf[10]};
                    hum_raw  <= {rx_buf[12], rx_buf[13], rx_buf[15], rx_buf[16]};

                    data_valid <= 1;
                    state      <= S_WAIT;  // Wait for next interval
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule