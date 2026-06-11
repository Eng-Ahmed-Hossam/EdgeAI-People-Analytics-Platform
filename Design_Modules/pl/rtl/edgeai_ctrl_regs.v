/*
------------------------------------------------------------------------------
Module Name : edgeai_ctrl_regs
Project     : EdgeAI Location Intelligence — Zynq Integration
Description :
    AXI4-Lite slave — PS-accessible control and status register bank
    for the EdgeAI CNN accelerator PL subsystem.

    Connected to PS via M_AXI_GP0 (32-bit, low-bandwidth control path).
    Base address configured in Vivado address editor.

    Register Map (byte address, 32-bit word aligned):
    ┌──────────┬────────┬───────────────────────────────────────────────┐
    │ Offset   │ R/W    │ Description                                   │
    ├──────────┼────────┼───────────────────────────────────────────────┤
    │ 0x00     │ W1S/RO │ CTRL: [0]=inference_start [1]=soft_reset      │
    │          │        │       [2]=tensor_clear                        │
    │ 0x04     │ RO     │ STATUS: [0]=busy [1]=tensor_ready             │
    │          │        │         [2]=inference_done [3]=cam_frame_error│
    │ 0x08     │ RW     │ WT_BASE_ADDR: PS DDR base of weight blob      │
    │ 0x0C     │ RW     │ BIAS_BASE_ADDR: PS DDR base of bias blob      │
    │ 0x10     │ RW     │ FRAME_BUF_A: PS DDR base of frame buffer A   │
    │ 0x14     │ RW     │ FRAME_BUF_B: PS DDR base of frame buffer B   │
    │ 0x18     │ RW     │ TENSOR_DST: PS DDR destination for tensor DMA│
    │ 0x1C     │ RW     │ LAYER_CTRL: [4:0]=current_layer_override     │
    └──────────┴────────┴───────────────────────────────────────────────┘

    CTRL writes are single-cycle pulses on the corresponding PL signals:
      inference_start  → TOP FSM input
      soft_reset       → synchronous reset for CNN subsystem
      tensor_clear     → clears tensor_output_buffer

    AXI4-Lite compliance:
      - Supports single 32-bit reads and writes (no bursts)
      - BRESP/RRESP always OKAY (2'b00)
      - Write response given after WVALID+WREADY handshake
------------------------------------------------------------------------------
*/

module edgeai_ctrl_regs #(
    parameter AXI_AW = 32,
    parameter AXI_DW = 32
)(
    input  wire        clk,
    input  wire        rst,

    // AXI4-Lite Slave — Write Address Channel
    input  wire [AXI_AW-1:0] s_axi_awaddr,
    input  wire              s_axi_awvalid,
    output reg               s_axi_awready,

    // AXI4-Lite Slave — Write Data Channel
    input  wire [AXI_DW-1:0] s_axi_wdata,
    input  wire [3:0]        s_axi_wstrb,
    input  wire              s_axi_wvalid,
    output reg               s_axi_wready,

    // AXI4-Lite Slave — Write Response Channel
    output reg  [1:0]        s_axi_bresp,
    output reg               s_axi_bvalid,
    input  wire              s_axi_bready,

    // AXI4-Lite Slave — Read Address Channel
    input  wire [AXI_AW-1:0] s_axi_araddr,
    input  wire              s_axi_arvalid,
    output reg               s_axi_arready,

    // AXI4-Lite Slave — Read Data Channel
    output reg  [AXI_DW-1:0] s_axi_rdata,
    output reg  [1:0]        s_axi_rresp,
    output reg               s_axi_rvalid,
    input  wire              s_axi_rready,

    // Output control pulses / registers to PL
    output reg         inference_start,   // 1-cycle pulse → TOP FSM
    output reg         cnn_soft_reset,    // 1-cycle pulse → synchronous reset path
    output reg         tensor_clear,      // 1-cycle pulse → tensor_output_buffer
    output wire [31:0] wt_base_addr,      // DDR base for weights
    output wire [31:0] bias_base_addr,    // DDR base for biases
    output wire [31:0] frame_buf_a,       // DDR frame buffer A base
    output wire [31:0] frame_buf_b,       // DDR frame buffer B base
    output wire [31:0] tensor_dst,        // DDR destination for tensor DMA

    // PL status inputs
    input  wire        busy,
    input  wire        tensor_ready,
    input  wire        inference_done,
    input  wire        cam_frame_error
);

    // =========================================================================
    // Registered outputs
    // =========================================================================
    reg [31:0] reg_wt_base;
    reg [31:0] reg_bias_base;
    reg [31:0] reg_frame_a;
    reg [31:0] reg_frame_b;
    reg [31:0] reg_tensor_dst;

    assign wt_base_addr  = reg_wt_base;
    assign bias_base_addr = reg_bias_base;
    assign frame_buf_a   = reg_frame_a;
    assign frame_buf_b   = reg_frame_b;
    assign tensor_dst    = reg_tensor_dst;

    // =========================================================================
    // Write channel FSM
    // =========================================================================
    reg [AXI_AW-1:0] wr_addr_latch;
    reg wr_addr_valid;
    reg wr_data_valid;
    reg [AXI_DW-1:0] wr_data_latch;

    always @(posedge clk) begin
        if (rst) begin
            s_axi_awready  <= 1'b1;
            s_axi_wready   <= 1'b1;
            s_axi_bvalid   <= 1'b0;
            s_axi_bresp    <= 2'b00;
            wr_addr_valid  <= 1'b0;
            wr_data_valid  <= 1'b0;
            inference_start <= 1'b0;
            cnn_soft_reset  <= 1'b0;
            tensor_clear    <= 1'b0;
            reg_wt_base    <= 32'h0021_0000;
            reg_bias_base  <= 32'h0044_0000;
            reg_frame_a    <= 32'h0020_0000;
            reg_frame_b    <= 32'h0020_8000;
            reg_tensor_dst <= 32'h0044_3000;
        end else begin
            // Default: deassert pulse outputs
            inference_start <= 1'b0;
            cnn_soft_reset  <= 1'b0;
            tensor_clear    <= 1'b0;

            // Accept write address
            if (s_axi_awvalid && s_axi_awready) begin
                wr_addr_latch <= s_axi_awaddr;
                wr_addr_valid <= 1'b1;
            end

            // Accept write data
            if (s_axi_wvalid && s_axi_wready) begin
                wr_data_latch <= s_axi_wdata;
                wr_data_valid <= 1'b1;
            end

            // Process write when both address and data are available
            if (wr_addr_valid && wr_data_valid) begin
                wr_addr_valid <= 1'b0;
                wr_data_valid <= 1'b0;

                case (wr_addr_latch[4:0])
                    5'h00: begin   // CTRL register (W1S)
                        if (wr_data_latch[0]) inference_start <= 1'b1;
                        if (wr_data_latch[1]) cnn_soft_reset  <= 1'b1;
                        if (wr_data_latch[2]) tensor_clear    <= 1'b1;
                    end
                    5'h08: reg_wt_base    <= apply_strobe(reg_wt_base,    wr_data_latch, s_axi_wstrb);
                    5'h0C: reg_bias_base  <= apply_strobe(reg_bias_base,  wr_data_latch, s_axi_wstrb);
                    5'h10: reg_frame_a    <= apply_strobe(reg_frame_a,    wr_data_latch, s_axi_wstrb);
                    5'h14: reg_frame_b    <= apply_strobe(reg_frame_b,    wr_data_latch, s_axi_wstrb);
                    5'h18: reg_tensor_dst <= apply_strobe(reg_tensor_dst, wr_data_latch, s_axi_wstrb);
                    default: ;
                endcase

                // Issue write response
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00;
            end

            if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // =========================================================================
    // Read channel
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            s_axi_arready <= 1'b1;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'h0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                s_axi_arready <= 1'b0;
                s_axi_rvalid  <= 1'b1;
                s_axi_rresp   <= 2'b00;

                case (s_axi_araddr[4:0])
                    5'h00: s_axi_rdata <= 32'h0;  // CTRL is write-only, read returns 0
                    5'h04: s_axi_rdata <= {28'd0,
                                           cam_frame_error,
                                           inference_done,
                                           tensor_ready,
                                           busy};
                    5'h08: s_axi_rdata <= reg_wt_base;
                    5'h0C: s_axi_rdata <= reg_bias_base;
                    5'h10: s_axi_rdata <= reg_frame_a;
                    5'h14: s_axi_rdata <= reg_frame_b;
                    5'h18: s_axi_rdata <= reg_tensor_dst;
                    default: s_axi_rdata <= 32'hDEADBEEF;
                endcase
            end

            if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid  <= 1'b0;
                s_axi_arready <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Byte-strobe helper function
    // =========================================================================
    function [31:0] apply_strobe;
        input [31:0] old_val;
        input [31:0] new_val;
        input [3:0]  strobe;
        begin
            apply_strobe[7:0]   = strobe[0] ? new_val[7:0]   : old_val[7:0];
            apply_strobe[15:8]  = strobe[1] ? new_val[15:8]  : old_val[15:8];
            apply_strobe[23:16] = strobe[2] ? new_val[23:16] : old_val[23:16];
            apply_strobe[31:24] = strobe[3] ? new_val[31:24] : old_val[31:24];
        end
    endfunction

endmodule
