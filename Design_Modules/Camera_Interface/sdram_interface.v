`timescale 1ns / 1ps

/*
 * sdram_interface.v
 *
 * Interface module between FPGA logic and SDRAM controller.
 * Manages burst read/write operations between asynchronous FIFOs (camera/VGA)
 * and the SDRAM controller.
 *
 * Inputs:
 * - clk          : system clock for SDRAM interface (165MHz)
 * - rst_n        : active-low reset
 * - clk_vga      : VGA clock domain for reading from FIFO
 * - rd_en        : read enable from VGA
 * - data_count_r : current number of words in FIFO (VGA side)
 * - f2s_data     : data to write to SDRAM
 *
 * Outputs:
 * - f2s_data_valid : asserted when data is valid for SDRAM write
 * - empty_fifo     : asserted when FIFO is empty
 * - dout           : data read from SDRAM (to FIFO)
 * - sdram signals  : clk, cke, cs_n, ras_n, cas_n, we_n, addr, bank, dq, dqm
 */

module sdram_interface(
    input clk, rst_n,
    // FIFO interface
    input wire clk_vga, rd_en,
    input wire [9:0] data_count_r,
    input wire [15:0] f2s_data,
    output wire f2s_data_valid,
    output wire empty_fifo,
    output wire [15:0] dout,
    // SDRAM interface
    output wire sdram_clk,
    output wire sdram_cke, 
    output wire sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n, 
    output wire [12:0] sdram_addr,
    output wire [1:0] sdram_ba, 
    output wire [1:0] sdram_dqm, 
    inout [15:0] sdram_dq
);

    // FSM states
    localparam IDLE      = 1'b0,
               BURST_OP  = 1'b1;

    // FSM registers
    reg state_q = IDLE, state_d;
    reg [14:0] wr_addr_q = 0, wr_addr_d;
    reg [14:0] rd_addr_q = 0, rd_addr_d;

    // SDRAM control signals
    reg rw, rw_en;
    reg [14:0] f_addr;

    // Signals from SDRAM controller
    wire [15:0] s2f_data;
    wire s2f_data_valid;
    wire ready;
    wire [9:0] data_count_w;

    // ----------------------
    // FSM registers update
    // ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q    <= IDLE;
            wr_addr_q  <= 0;
            rd_addr_q  <= 0;
        end else begin
            state_q    <= state_d;
            wr_addr_q  <= wr_addr_d;
            rd_addr_q  <= rd_addr_d;
        end
    end

    // ----------------------
    // FSM next-state logic
    // ----------------------
    always @(*) begin
        // Defaults
        state_d    = state_q;
        wr_addr_d  = wr_addr_q;
        rd_addr_d  = rd_addr_q;
        f_addr     = 0;
        rw         = 0;
        rw_en      = 0;

        case(state_q)
            IDLE: begin
                if (data_count_r > 512 && ready) begin
                    // Enough FIFO data to burst-write
                    rw_en      = 1;
                    rw         = 0; // write
                    wr_addr_d  = wr_addr_q;
                    f_addr     = wr_addr_q;
                    state_d    = BURST_OP;
                end
            end
            BURST_OP: begin
                if (ready) begin
                    if (data_count_r > 512) begin
                        // Burst write to SDRAM
                        rw_en      = 1;
                        rw         = 0;
                        wr_addr_d  = (wr_addr_q == 599) ? 0 : wr_addr_q + 1;
                        f_addr     = wr_addr_q;
                    end else if (data_count_w < 250) begin
                        // Burst read from SDRAM to fill FIFO for VGA
                        rw_en      = 1;
                        rw         = 1;
                        rd_addr_d  = (rd_addr_q == 599) ? 0 : rd_addr_q + 1;
                        f_addr     = rd_addr_q;
                    end
                end
            end
            default: state_d = IDLE;
        endcase
    end

    // ----------------------
    // SDRAM controller instance
    // ----------------------
    sdram_controller u_sdram_controller (
        .clk(clk),
        .rst_n(rst_n),
        .rw(rw),
        .rw_en(rw_en),
        .f_addr(f_addr),
        .f2s_data(f2s_data),
        .s2f_data(s2f_data),
        .s2f_data_valid(s2f_data_valid),
        .f2s_data_valid(f2s_data_valid),
        .ready(ready),
        .s_clk(sdram_clk),
        .s_cke(sdram_cke),
        .s_cs_n(sdram_cs_n),
        .s_ras_n(sdram_ras_n),
        .s_cas_n(sdram_cas_n),
        .s_we_n(sdram_we_n),
        .s_addr(sdram_addr),
        .s_ba(sdram_ba),
        .LDQM(sdram_dqm[0]),
        .HDQM(sdram_dqm[1]),
        .s_dq(sdram_dq)
    );

    // ----------------------
    // Asynchronous FIFO instance (camera/VGA interface)
    // ----------------------
    asyn_fifo #(
        .DATA_WIDTH(16),
        .FIFO_DEPTH_WIDTH(10) // 1024x16 FIFO
    ) u_fifo (
        .rst_n(rst_n),
        .clk_write(clk),
        .clk_read(clk_vga),
        .write(s2f_data_valid),
        .read(rd_en),
        .data_write(s2f_data),
        .data_read(dout),
        .full(),
        .empty(empty_fifo),
        .data_count_w(data_count_w)
    );

endmodule
