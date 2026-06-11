// =============================================================================
// Module: sensor_hub (TOP MODULE)
// Description: Integrates the SCD30 and BH1750 sensor drivers onto separate
//              I2C buses (they share different addresses, but using separate
//              buses avoids bus contention and simplifies timing).
//              Exposes a unified data-valid interface and register bank.
//
// Output Summary:
//   co2_raw   – IEEE 754 float (32-bit): CO2 in ppm
//   temp_raw  – IEEE 754 float (32-bit): Temperature in °C
//   hum_raw   – IEEE 754 float (32-bit): Relative Humidity in %
//   lux_x10   – Fixed-point integer × 10: Lux (e.g. 1234 = 123.4 lux)
//   raw_count – Raw ADC count from BH1750
// =============================================================================

module sensor_hub (
    // ── System ──────────────────────────────────────────────────────────────
    input  wire        clk,             // 50 MHz system clock
    input  wire        rst_n,           // Active-low synchronous reset

    // ── Control ──────────────────────────────────────────────────────────────
    input  wire        enable,          // 1 = run both sensors
    input  wire [15:0] scd30_interval,  // Measurement interval (ms, e.g. 2000)
    input  wire        bh1750_addr_sel, // 0 = 0x23, 1 = 0x5C

    // ── SCD30 Outputs ────────────────────────────────────────────────────────
    output wire [31:0] co2_raw,
    output wire [31:0] temp_raw,
    output wire [31:0] hum_raw,
    output wire        scd30_valid,     // New SCD30 data ready
    output wire        scd30_crc_err,

    // ── BH1750 Outputs ───────────────────────────────────────────────────────
    output wire [15:0] lux_x10,
    output wire [15:0] bh1750_raw,
    output wire        bh1750_valid,    // New BH1750 data ready

    // ── I2C Bus A: SCD30 ─────────────────────────────────────────────────────
    output wire        scd30_scl,
    inout  wire        scd30_sda,

    // ── I2C Bus B: BH1750 ────────────────────────────────────────────────────
    output wire        bh1750_scl,
    inout  wire        bh1750_sda
);

    // ── SCD30 Driver Instance ────────────────────────────────────────────
    scd30_driver u_scd30 (
        .clk         (clk),
        .rst_n       (rst_n),
        .enable      (enable),
        .interval_ms (scd30_interval),
        .data_valid  (scd30_valid),
        .co2_raw     (co2_raw),
        .temp_raw    (temp_raw),
        .hum_raw     (hum_raw),
        .crc_error   (scd30_crc_err),
        .sda         (scd30_sda),
        .scl         (scd30_scl)
    );

    // ── BH1750 Driver Instance ───────────────────────────────────────────
    bh1750_driver u_bh1750 (
        .clk         (clk),
        .rst_n       (rst_n),
        .enable      (enable),
        .addr_sel    (bh1750_addr_sel),
        .data_valid  (bh1750_valid),
        .lux_x10     (lux_x10),
        .raw_count   (bh1750_raw),
        .sda         (bh1750_sda),
        .scl         (bh1750_scl)
    );

endmodule
