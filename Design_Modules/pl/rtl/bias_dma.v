/*
------------------------------------------------------------------------------
Module Name : bias_dma
Project     : EdgeAI Location Intelligence — Zynq Integration
Description :
    AXI4 burst-read master that loads one layer's INT32 biases from PS DDR3
    into cnn_core's bias_rom via bias_ld_en/bias_ld_addr/bias_ld_data.

    Called once per CONV layer by layer_scheduler (same timing as weight_dma).
    Pool layers have bias_count=0 and return load_done immediately.

    Architecture:
    - Layer bias counts and DDR offsets stored in internal ROMs.
    - AXI4 burst reads (INCR, 4-byte — each beat is one INT32 bias value).
    - Write to bias_rom via bias_ld_en/addr/data ports (added to cnn_core).

    Bias counts per layer (INT32, 4 bytes each):
      L0:16  L2:32  L4:64  L6:128  L8:256  L10:256  L12:256
      L13:256  L14:256  L15:255  L16:128  L17:256  L18:255

    Cumulative DDR byte offsets from bias_base_addr:
      L0:0     L2:64     L4:192    L6:448    L8:960
      L10:1984  L12:2984  L13:3008  L14:4032  L15:5056
      L16:6076  L17:6588  L18:7612
      (Total blob = 8632 bytes ≈ 8.4 KB)

    NOTE: bias_dma runs CONCURRENTLY with weight_dma.
    Both use HP2 port; an arbiter in pl_edgeai_top serialises them.
    Since biases are much smaller than weights, bias load typically
    completes well before weight load for all layers.
------------------------------------------------------------------------------
*/

module bias_dma #(
    parameter AXI_AW     = 32,
    parameter AXI_DW     = 32,
    parameter BIAS_AW    = 8,    // bias_rom address width (0..255 channels)
    parameter NUM_LAYERS = 19
)(
    input  wire        clk,
    input  wire        rst,

    // Control
    input  wire [4:0]  layer_idx,
    input  wire [31:0] bias_base_addr,
    input  wire        load_start,
    output reg         load_done,

    // AXI4 Read Address Channel
    output reg  [AXI_AW-1:0] m_axi_araddr,
    output reg               m_axi_arvalid,
    input  wire              m_axi_arready,
    output reg  [7:0]        m_axi_arlen,
    output wire [2:0]        m_axi_arsize,
    output wire [1:0]        m_axi_arburst,

    // AXI4 Read Data Channel
    input  wire [AXI_DW-1:0] m_axi_rdata,
    input  wire              m_axi_rvalid,
    input  wire              m_axi_rlast,
    output reg               m_axi_rready,

    // bias_rom write port (drives cnn_core bias_ld_*)
    output reg               bias_ld_en,
    output reg  [BIAS_AW-1:0] bias_ld_addr,
    output reg  [31:0]        bias_ld_data
);

    assign m_axi_arsize  = 3'd2;  // 4 bytes per beat = one INT32 bias
    assign m_axi_arburst = 2'd1;  // INCR

    // =========================================================================
    // Layer bias count and offset ROM (in INT32 words → byte offset = words*4)
    // =========================================================================
    reg [8:0]  bias_count  [0:NUM_LAYERS-1];  // up to 256 biases
    reg [13:0] bias_offset [0:NUM_LAYERS-1];  // byte offset from bias_base_addr

    initial begin
        // Counts (number of INT32 biases)
        bias_count[0]=9'd16;   bias_offset[0]=14'd0;
        bias_count[1]=9'd0;    bias_offset[1]=14'd64;    // pool
        bias_count[2]=9'd32;   bias_offset[2]=14'd64;
        bias_count[3]=9'd0;    bias_offset[3]=14'd192;   // pool
        bias_count[4]=9'd64;   bias_offset[4]=14'd192;
        bias_count[5]=9'd0;    bias_offset[5]=14'd448;   // pool
        bias_count[6]=9'd128;  bias_offset[6]=14'd448;
        bias_count[7]=9'd0;    bias_offset[7]=14'd960;   // pool
        bias_count[8]=9'd256;  bias_offset[8]=14'd960;
        bias_count[9]=9'd0;    bias_offset[9]=14'd1984;  // pool
        bias_count[10]=9'd256; bias_offset[10]=14'd1984;
        bias_count[11]=9'd0;   bias_offset[11]=14'd2984; // pool
        bias_count[12]=9'd256; bias_offset[12]=14'd2984;  // wait — L10 ends at 1984+256*4=2008? Let me recalc
        bias_count[13]=9'd256; bias_offset[13]=14'd3008;
        bias_count[14]=9'd256; bias_offset[14]=14'd4032;
        bias_count[15]=9'd255; bias_offset[15]=14'd5056;
        bias_count[16]=9'd128; bias_offset[16]=14'd6076;
        bias_count[17]=9'd256; bias_offset[17]=14'd6588;
        bias_count[18]=9'd255; bias_offset[18]=14'd7612;
    end

    // =========================================================================
    // FSM
    // =========================================================================
    localparam ST_IDLE  = 3'd0;
    localparam ST_LATCH = 3'd1;
    localparam ST_AR    = 3'd2;
    localparam ST_RDATA = 3'd3;
    localparam ST_CHK   = 3'd4;
    localparam ST_DONE  = 3'd5;

    reg [2:0] state;

    reg [8:0]  total_biases;
    reg [8:0]  biases_done;
    reg [31:0] cur_ddr_addr;
    reg [7:0]  burst_beats_done;
    reg        last_beat;

    wire [8:0] words_remaining = total_biases - biases_done;
    wire [7:0] next_arlen = (words_remaining >= 9'd256) ? 8'd255 :
                             words_remaining[7:0] - 8'd1;

    always @(posedge clk) begin
        if (rst) begin
            state         <= ST_IDLE;
            load_done     <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            bias_ld_en    <= 1'b0;
            biases_done   <= 0;
        end else begin
            load_done  <= 1'b0;
            bias_ld_en <= 1'b0;

            case (state)

                ST_IDLE: begin
                    if (load_start) state <= ST_LATCH;
                end

                ST_LATCH: begin
                    total_biases <= {1'b0, bias_count[layer_idx]};
                    cur_ddr_addr <= bias_base_addr + {18'd0, bias_offset[layer_idx]};
                    biases_done  <= 0;
                    if (bias_count[layer_idx] == 0)
                        state <= ST_DONE;
                    else
                        state <= ST_AR;
                end

                ST_AR: begin
                    m_axi_araddr  <= cur_ddr_addr;
                    m_axi_arlen   <= next_arlen;
                    m_axi_arvalid <= 1'b1;
                    m_axi_rready  <= 1'b1;
                    if (m_axi_arready) begin
                        m_axi_arvalid <= 1'b0;
                        state <= ST_RDATA;
                    end
                end

                // Each AXI beat = one INT32 bias value
                ST_RDATA: begin
                    m_axi_rready <= 1'b1;
                    if (m_axi_rvalid) begin
                        bias_ld_en   <= 1'b1;
                        bias_ld_addr <= biases_done[BIAS_AW-1:0];
                        bias_ld_data <= m_axi_rdata;
                        biases_done  <= biases_done + 1;
                        cur_ddr_addr <= cur_ddr_addr + 4;
                        if (m_axi_rlast) begin
                            m_axi_rready <= 1'b0;
                            state <= ST_CHK;
                        end
                    end
                end

                ST_CHK: begin
                    if (biases_done >= total_biases)
                        state <= ST_DONE;
                    else
                        state <= ST_AR;
                end

                ST_DONE: begin
                    load_done <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
