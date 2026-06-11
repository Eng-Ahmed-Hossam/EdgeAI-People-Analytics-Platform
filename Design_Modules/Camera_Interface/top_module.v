`timescale 1ns / 1ps

/*
 * top_module.v
 *
 * Top-level module integrating:
 * - camera interface
 * - SDRAM interface
 * - VGA interface
 *
 * Handles the pipeline: Camera → FIFO → SDRAM → VGA
 */

module top_module(
    input  wire clk,
    input  wire rst_n,
    input  wire [3:0] key,            // key[1:0]: brightness, key[3:2]: contrast

    // Camera pinouts
    input  wire cmos_pclk,
    input  wire cmos_href,
    input  wire cmos_vsync,
    input  wire [7:0] cmos_db,
    inout  wire cmos_sda,
    inout  wire cmos_scl,
    output wire cmos_rst_n,
    output wire cmos_pwdn,
    output wire cmos_xclk,

    // Debug LEDs
    output wire [3:0] led,

    // SDRAM interface
    output wire sdram_clk,
    output wire sdram_cke,
    output wire sdram_cs_n,
    output wire sdram_ras_n,
    output wire sdram_cas_n,
    output wire sdram_we_n,
    output wire [12:0] sdram_addr,
    output wire [1:0] sdram_ba,
    output wire [1:0] sdram_dqm,
    inout  wire [15:0] sdram_dq,

    // VGA output
    output wire [4:0] vga_out_r,
    output wire [5:0] vga_out_g,
    output wire [4:0] vga_out_b,
    output wire vga_out_vs,
    output wire vga_out_hs
);

    // --------------------------------
    // Internal wires
    // --------------------------------
    wire        f2s_data_valid;
    wire [9:0]  data_count_r;
    wire [15:0] dout, din;
    wire        clk_sdram;
    wire        empty_fifo;
    wire        clk_vga;
    wire        rd_en;

    // --------------------------------
    // Camera interface
    // --------------------------------
    camera_interface u_camera (
        .clk(clk),
        .clk_100(clk_sdram),
        .rst_n(rst_n),
        .key(key),
        .rd_en(f2s_data_valid),
        .data_count_r(data_count_r),
        .dout(dout),
        .cmos_pclk(cmos_pclk),
        .cmos_href(cmos_href),
        .cmos_vsync(cmos_vsync),
        .cmos_db(cmos_db),
        .cmos_sda(cmos_sda),
        .cmos_scl(cmos_scl),
        .cmos_rst_n(cmos_rst_n),
        .cmos_pwdn(cmos_pwdn),
        .cmos_xclk(cmos_xclk),
        .led(led)
    );

    // --------------------------------
    // SDRAM interface
    // --------------------------------
    sdram_interface u_sdram (
        .clk(clk_sdram),
        .rst_n(rst_n),
        .clk_vga(clk_vga),
        .rd_en(rd_en),
        .data_count_r(data_count_r),
        .f2s_data(dout),
        .f2s_data_valid(f2s_data_valid),
        .empty_fifo(empty_fifo),
        .dout(din),
        .sdram_cke(sdram_cke),
        .sdram_cs_n(sdram_cs_n),
        .sdram_ras_n(sdram_ras_n),
        .sdram_cas_n(sdram_cas_n),
        .sdram_we_n(sdram_we_n),
        .sdram_addr(sdram_addr),
        .sdram_ba(sdram_ba),
        .sdram_dqm(sdram_dqm),
        .sdram_dq(sdram_dq)
    );

    // --------------------------------
    // VGA interface
    // --------------------------------
    vga_interface u_vga (
        .clk(clk),
        .rst_n(rst_n),
        .empty_fifo(empty_fifo),
        .din(din),
        .clk_vga(clk_vga),
        .rd_en(rd_en),
        .vga_out_r(vga_out_r),
        .vga_out_g(vga_out_g),
        .vga_out_b(vga_out_b),
        .vga_out_vs(vga_out_vs),
        .vga_out_hs(vga_out_hs)
    );

    // --------------------------------
    // SDRAM output DDR clock generation
    // --------------------------------
    ODDR2 #(
        .DDR_ALIGNMENT("NONE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) u_oddr2 (
        .D0(1'b0),
        .D1(1'b1),
        .C0(clk_sdram),
        .C1(~clk_sdram),
        .CE(1'b1),
        .R(1'b0),
        .S(1'b0),
        .Q(sdram_clk)
    );

    // --------------------------------
    // SDRAM clock generation
    // --------------------------------
    dcm_165MHz u_dcm (
        .clk(clk),
        .clk_sdram(clk_sdram),
        .RESET(~rst_n),
        .LOCKED() // optional
    );

endmodule
