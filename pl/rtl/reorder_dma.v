/*
------------------------------------------------------------------------------
Module Name : reorder_dma
Project     : EdgeAI Location Intelligence — Zynq Integration
Description :
    Captures the cnn_core output stream (tile-major order) and writes it
    into ifmap_buffer in channel-major order, ready for the next layer.

    The reorder transform was proven byte-exact in tb_chained.sv:
      Source order:  [oc_tile][y][x][row]  (cnn_core output stream)
      Target layout: [ch][y][x]            (ifmap_buffer channel-major)

      ch   = tile * PE_ROWS + row
      addr = ch * H * W + y * W + x

    Architecture — two-phase per layer:

      PHASE 1 (capture): While cnn_core computes, each out_valid byte is
        written sequentially to an internal staging BRAM (STAGE_MEM).
        cnn_core reads ifmap_buffer during this phase; staging BRAM is a
        separate memory, so no conflict occurs.

      PHASE 2 (scatter): After compute_done (reorder_start pulse), the AGU
        reads from staging BRAM and scatter-writes to ifmap_buffer via the
        ifmap_ext_wr_* port, computing the channel-major address on-the-fly.
        cnn_core is idle during this phase.

    The two-phase design exactly mirrors the testbench approach (cap[] array
    in tb_chained.sv = STAGE_MEM, reorder_conv_to_ifmap task = scatter phase).

    Pool layer bypass (pool_mode=1):
      Pool output is already channel-major. PHASE 1 capture is the same;
      PHASE 2 writes sequentially (no scatter), address = sequential index.

    Last layer bypass (last_layer=1):
      Stream goes directly to tensor_output_buffer via mux in pl_edgeai_top.
      reorder_dma does nothing; reorder_done asserted immediately.

    Staging BRAM sizing:
      Maximum output = L0: 16×128×128 = 262,144 bytes → STAGE_DEPTH = 262144
      On Zynq Z-7020 (220 BRAM36 = 8.2 Mb): 262144×8 = 2.1 Mb = ~26% BRAM.

    Parameters:
      PE_ROWS     — must match cnn_core PE_ROWS (default 16)
      MAX_OUT     — max output bytes per layer (default 262144 for L0)
      STAGE_AW    — log2(MAX_OUT) = 18
      IFMAP_AW    — ifmap_buffer address width (18 for tb_chained sizing)
------------------------------------------------------------------------------
*/

module reorder_dma #(
    parameter DATA_WIDTH = 8,
    parameter PE_ROWS    = 16,
    parameter MAX_OUT    = 262144,    // Largest layer output (L0: 16×128×128)
    parameter STAGE_AW   = 18,        // log2(262144)
    parameter IFMAP_AW   = 18,        // ifmap_buffer address width
    parameter MAX_DIM    = 128,
    parameter MAX_CH     = 256
)(
    input  wire        clk,
    input  wire        rst,

    // ----------------------------------------------------------------
    // Layer configuration (from layer_scheduler, stable before compute)
    // ----------------------------------------------------------------
    input  wire [8:0]  out_channels,  // Number of output channels for this layer
    input  wire [7:0]  out_dim,       // Spatial dimension H=W of this layer's output
    input  wire        pool_mode,     // 1 = pool output (already ch-major, no scatter)
    input  wire        last_layer,    // 1 = L18 (route to tensor buf, skip reorder)

    // ----------------------------------------------------------------
    // Phase 1: capture — cnn_core output stream
    // ----------------------------------------------------------------
    input  wire signed [DATA_WIDTH-1:0] stream_data,
    input  wire        stream_valid,
    input  wire        stream_last,   // asserted when compute_done (CS_DONE)

    // ----------------------------------------------------------------
    // Phase 2: scatter — triggered after compute_done
    // ----------------------------------------------------------------
    input  wire        reorder_start,  // Pulse: begin scatter phase (= compute_done)
    output reg         reorder_done,   // Pulse: scatter complete

    // ----------------------------------------------------------------
    // ifmap_buffer external write port
    // ----------------------------------------------------------------
    output reg                          ifmap_wr_en,
    output reg  [IFMAP_AW-1:0]         ifmap_wr_addr,
    output reg  signed [DATA_WIDTH-1:0] ifmap_wr_data

    // NOTE: tensor path is a separate mux in pl_edgeai_top; this module
    // does NOT drive tensor_output_buffer directly.
);

    // =========================================================================
    // Staging BRAM
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] stage_mem [0:MAX_OUT-1];
    reg [STAGE_AW-1:0] stage_wr_ptr;   // Phase 1 write pointer
    reg [STAGE_AW-1:0] stage_rd_ptr;   // Phase 2 read pointer
    reg [STAGE_AW-1:0] stage_total;    // Total bytes captured in phase 1

    // Phase 1 capture: always active for non-last-layer streams
    always @(posedge clk) begin
        if (rst || reorder_start) begin
            stage_wr_ptr <= 0;
        end else if (stream_valid && !last_layer) begin
            stage_mem[stage_wr_ptr] <= stream_data;
            stage_wr_ptr <= stage_wr_ptr + 1;
        end
    end

    // Latch total count at end of stream
    always @(posedge clk) begin
        if (rst)
            stage_total <= 0;
        else if (stream_last && !last_layer)
            stage_total <= stage_wr_ptr + (stream_valid ? 1 : 0);
    end

    // =========================================================================
    // Phase 2 FSM — scatter from staging BRAM to ifmap_buffer
    // =========================================================================
    localparam PH_IDLE    = 3'd0;
    localparam PH_SETUP   = 3'd1;   // Latch counters
    localparam PH_READ    = 3'd2;   // Read from stage_mem
    localparam PH_WRITE   = 3'd3;   // Write to ifmap (1 cycle after read)
    localparam PH_ADVANCE = 3'd4;   // Advance AGU counters
    localparam PH_DONE    = 3'd5;

    reg [2:0] ph_state;

    // AGU counters (tile-major order: tile → y → x → row)
    reg [3:0]  agu_tile;
    reg [7:0]  agu_y;
    reg [7:0]  agu_x;
    reg [3:0]  agu_row;

    reg [4:0]  agu_ntiles;       // ceil(out_ch / PE_ROWS)
    reg [4:0]  agu_active;       // active rows in current tile
    reg [15:0] agu_dimsq;        // out_dim * out_dim

    reg [STAGE_AW-1:0] scatter_idx;  // index into stage_mem

    // Latched config for scatter phase
    reg [8:0]  latch_out_ch;
    reg [7:0]  latch_out_dim;
    reg        latch_pool_mode;

    // Compute active rows for current tile
    wire [8:0] tile_base = {5'd0, agu_tile} * PE_ROWS;
    wire [8:0] remaining = latch_out_ch - tile_base;
    wire [4:0] active_rows = (remaining >= 9'd16) ? 5'd16 : remaining[4:0];

    // Scatter address: ch * dim² + y * dim + x
    wire [8:0]  ch_scatter = tile_base[8:0] + {5'd0, agu_row};
    wire [31:0] scatter_addr_wide = ({23'd0, ch_scatter} * {16'd0, agu_dimsq}) +
                                    ({24'd0, agu_y} * {24'd0, latch_out_dim}) +
                                    {24'd0, agu_x};

    // Stage memory read data (1-cycle latency)
    reg signed [DATA_WIDTH-1:0] stage_rdata;
    always @(posedge clk) begin
        stage_rdata <= stage_mem[stage_rd_ptr];
    end

    always @(posedge clk) begin
        if (rst) begin
            ph_state     <= PH_IDLE;
            reorder_done <= 1'b0;
            ifmap_wr_en  <= 1'b0;
            scatter_idx  <= 0;
            stage_rd_ptr <= 0;
        end else begin
            reorder_done <= 1'b0;
            ifmap_wr_en  <= 1'b0;

            case (ph_state)

                PH_IDLE: begin
                    if (reorder_start) begin
                        if (last_layer) begin
                            // Last layer: cnn_core output goes to tensor buffer via
                            // mux in pl_edgeai_top. Nothing for reorder_dma to do.
                            reorder_done <= 1'b1;
                        end else begin
                            ph_state <= PH_SETUP;
                        end
                    end
                end

                PH_SETUP: begin
                    // Latch layer config (stable at reorder_start)
                    latch_out_ch  <= out_channels;
                    latch_out_dim <= out_dim;
                    latch_pool_mode <= pool_mode;
                    agu_tile      <= 0;
                    agu_y         <= 0;
                    agu_x         <= 0;
                    agu_row       <= 0;
                    agu_ntiles    <= (out_channels + PE_ROWS - 1) >> $clog2(PE_ROWS);
                    agu_dimsq     <= {8'd0, out_dim} * {8'd0, out_dim};
                    scatter_idx   <= 0;
                    stage_rd_ptr  <= 0;
                    ph_state      <= PH_READ;
                end

                PH_READ: begin
                    // Issue read from staging BRAM (1-cycle latency)
                    stage_rd_ptr <= scatter_idx;
                    ph_state     <= PH_WRITE;
                end

                PH_WRITE: begin
                    // Write stage_rdata to ifmap_buffer at scatter address
                    ifmap_wr_en  <= 1'b1;
                    ifmap_wr_data <= stage_rdata;

                    if (latch_pool_mode) begin
                        // Pool output already channel-major: sequential address
                        ifmap_wr_addr <= scatter_idx[IFMAP_AW-1:0];
                    end else begin
                        // Conv scatter address
                        ifmap_wr_addr <= scatter_addr_wide[IFMAP_AW-1:0];
                    end

                    ph_state <= PH_ADVANCE;
                end

                PH_ADVANCE: begin
                    scatter_idx <= scatter_idx + 1;

                    if (!latch_pool_mode) begin
                        // Advance AGU: innermost = row, then x, then y, then tile
                        if (agu_row < active_rows - 1) begin
                            agu_row <= agu_row + 1;
                        end else begin
                            agu_row <= 0;
                            if (agu_x < latch_out_dim - 1) begin
                                agu_x <= agu_x + 1;
                            end else begin
                                agu_x <= 0;
                                if (agu_y < latch_out_dim - 1) begin
                                    agu_y <= agu_y + 1;
                                end else begin
                                    agu_y <= 0;
                                    agu_tile <= agu_tile + 1;
                                    // active_rows for the new tile will be recomputed
                                    // combinatorially next cycle using updated agu_tile
                                end
                            end
                        end
                    end

                    // Check completion
                    if (scatter_idx + 1 >= stage_total) begin
                        ph_state <= PH_DONE;
                    end else begin
                        ph_state <= PH_READ;
                    end
                end

                PH_DONE: begin
                    reorder_done <= 1'b1;
                    ph_state     <= PH_IDLE;
                end

                default: ph_state <= PH_IDLE;
            endcase
        end
    end

endmodule
