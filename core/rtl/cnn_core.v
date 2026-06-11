/*
------------------------------------------------------------------------------
Module Name : cnn_core
Project     : EdgeAI Location Intelligence
Description :
    Corrected CNN compute engine with output-stationary dataflow.

    Compute loop nest (Phase 6+7 fix):
        for oc_tile in 0 .. ceil(out_ch / PE_ROWS) - 1:
            for oy in 0 .. img_dim - 1:
                for ox in 0 .. img_dim - 1:
                    clear PE accumulators
                    for ic in 0 .. in_channels - 1:
                        for ky in 0 .. kernel_size - 1:
                            for kx in 0 .. kernel_size - 1:
                                if in_bounds: read ifmap[ic, oy+ky-pad, ox+kx-pad]
                                load weight_reg[0..15] from weight buffer (PE_ROWS weights)
                                MAC: PE[r][0] += ifmap × weight_reg[r]  for all r
                    for r in 0 .. active_rows - 1:
                        feed (pe_acc[r] + bias[oc_base+r]) through rq → act → pool
                        write result to out_data stream

    PE array used in SINGLE-COLUMN mode (col_en = 16'h0001):
        PE[r][0] accumulates the correct convolution result for output channel
        (oc_base + r).  Columns 1-15 always receive zeros and are disabled via
        col_en so they don't affect the accumulator.

    Weight buffer MUST be loaded in TILED format (layer<N>_weights.hex):
        for each oc_tile:
            for ic, ky, kx:
                PE_ROWS consecutive entries (one per output channel in tile)

    ifmap buffer layout: [ch][y][x] planar, BUF_DEPTH_IFMAP = 65536.

    Post-processing pipeline latency: 5 cycles
        bias_add(1) + requantizer_s1(1) + requantizer_s2(1) + activation(1) + pool_bypass(1)

    Output stream order: [oc_tile][oy][ox][row] — for a single oc_tile this is
        equivalent to [oy][ox][oc], matching layer0_expected_rtl_order.hex.

    Preserved sub-modules (verified correct, untouched):
        pe.v, pe_array.v, bias_add.v, requantizer.v, activation_unit.v,
        pooling_unit.v, weight_buffer.v, ifmap_buffer.v, ofmap_buffer.v

Change Log:
    v2.0 - Phases 6+7: correct loop nest, PE weight distribution, bias indexing
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module cnn_core #(
    parameter DATA_WIDTH      = `DATA_WIDTH,
    parameter ACC_WIDTH       = `ACC_WIDTH,
    parameter PE_ROWS         = `PE_ROWS,
    parameter PE_COLS         = `PE_COLS,
    parameter MAX_K           = `MAX_KERNEL_SIZE,
    parameter MAX_DIM         = `MAX_IMG_DIM,
    parameter MAX_CH          = `MAX_CHANNELS,
    // ifmap buffer: large enough for the biggest input layer
    // L2 = 16 ch × 64 × 64 = 65536 bytes  (largest after L0 = 3×128×128 = 49152)
    parameter BUF_DEPTH_IFMAP = 65536,
    parameter BUF_AW_IFMAP    = 16,
    // ofmap buffer: large enough for the biggest output layer
    // L0 = 16 ch × 128 × 128 = 262144 bytes
    parameter BUF_DEPTH_OFMAP = 262144,
    parameter BUF_AW_OFMAP    = 18,
    parameter WB_DEPTH        = 65536,
    parameter WB_AW           = $clog2(WB_DEPTH)
)(
    input  wire        clk,
    input  wire        rst,

    // Layer configuration (from layer_scheduler)
    input  wire [8:0]  in_channels,
    input  wire [8:0]  out_channels,
    input  wire [1:0]  kernel_size,      // Actual value: 1 or 3
    input  wire        stride,           // 0 = stride-1,  1 = stride-2 (all conv = 0)
    input  wire        pad_en,           // Zero-pad enable
    input  wire [1:0]  act_mode,
    input  wire        pool_en,
    input  wire [7:0]  img_dim,          // Spatial dimension (1..128)
    input  wire        layer_type,       // LAYER_TYPE_CONV or LAYER_TYPE_POOL

    // Requantization parameters
    input  wire [`MULT_WIDTH-1:0]       rq_multiplier,
    input  wire [`SHIFT_WIDTH-1:0]      rq_shift,
    input  wire signed [DATA_WIDTH-1:0] rq_zero_point,

    // Compute control
    input  wire        compute_start,
    output reg         compute_done,

    // Weight load interface (external → weight_buffer)
    input  wire                          wt_ld_en,
    input  wire [WB_AW-1:0]             wt_ld_addr,
    input  wire signed [DATA_WIDTH-1:0]  wt_ld_data,
    input  wire                          wt_swap,

    // ifmap write interface (from cnn_input_adapter or direct test load)
    input  wire                          ifmap_ext_wr_en,
    input  wire [BUF_AW_IFMAP-1:0]      ifmap_ext_wr_addr,
    input  wire signed [DATA_WIDTH-1:0]  ifmap_ext_wr_data,

    // Bias load interface — INTEGRATION ADDITION (compute logic unchanged)
    // Allows bias_dma to write per-layer biases from PS DDR at runtime.
    // In simulation, testbench may still use hierarchical access for bias_rom.
    input  wire        bias_ld_en,
    input  wire [7:0]  bias_ld_addr,   // Output channel index (0..MAX_CH-1)
    input  wire [31:0] bias_ld_data,   // INT32 bias value

    // Output stream (to tensor_output_buffer or testbench capture)
    output wire signed [DATA_WIDTH-1:0]  out_data,
    output wire                          out_valid,
    output wire                          out_last
);

    // =========================================================================
    // Bias ROM: INT32 per output channel.
    // In simulation: loaded by testbench via hierarchical reference.
    // In hardware: written by bias_dma via bias_ld_en/addr/data ports below.
    // =========================================================================
    localparam BIAS_DEPTH = MAX_CH;
    reg signed [`BIAS_WIDTH-1:0] bias_rom [0:BIAS_DEPTH-1];

    // Bias load write port (integration addition — no effect on compute path)
    always @(posedge clk) begin
        if (bias_ld_en)
            bias_rom[bias_ld_addr] <= $signed(bias_ld_data);
    end

    // =========================================================================
    // FSM state encoding
    // =========================================================================
    localparam CS_IDLE        = 4'd0;
    localparam CS_CLEAR       = 4'd1;   // Assert acc_clear for 1 cycle
    localparam CS_KER_REQ     = 4'd2;   // Issue ifmap + weight[0] reads
    localparam CS_KER_WT      = 4'd3;   // Collect PE_ROWS weights (+ ifmap @cnt=0)
    localparam CS_KER_MAC     = 4'd4;   // Pulse pe_en for 1 MAC cycle
    localparam CS_KER_NEXT    = 4'd5;   // Advance (kx→ky→ic); update wt_ker_base
    localparam CS_POST_FEED   = 4'd6;   // Feed PE row accumulators to post-proc
    localparam CS_POST_DRAIN  = 4'd7;   // Drain post-proc pipeline (POST_LATENCY cycles)
    localparam CS_SPATIAL_ADV = 4'd8;   // Advance ox/oy; loop or go to OC_ADV
    localparam CS_OC_ADV      = 4'd9;   // Advance oc_tile
    localparam CS_DONE        = 4'd10;  // Assert compute_done
    // Pool layer states (no PE involvement — reads ifmap directly into pooling_unit)
    localparam CS_POOL_SYNC  = 4'd11;  // Pulse frame_sync to pooling_unit; reset px
    localparam CS_POOL_RUN   = 4'd12;  // Sequential read from ifmap → pooling_unit
    localparam CS_POOL_DRAIN = 4'd13;  // Wait for last pool output to propagate
    localparam CS_POOL_NEXT  = 4'd14;  // Advance channel or go to CS_DONE

    // Pipeline latency: bias_add(1) + rq_s1(1) + rq_s2(1) + act(1) + pool_bypass(1) = 5
    localparam POST_LATENCY   = 5;

    reg [3:0] cs_state;

    // =========================================================================
    // Loop counters
    // =========================================================================
    reg [3:0]  oc_tile;         // Current output channel tile index
    reg [4:0]  oc_tile_max;     // Total output channel tiles = ceil(out_ch / PE_ROWS)
    reg [8:0]  oc_base;         // oc_tile * PE_ROWS
    reg [7:0]  oy, ox;          // Spatial output coordinates (0 .. img_dim-1)
    reg [8:0]  ic;              // Input channel counter (0 .. in_channels-1)
    reg [1:0]  ky, kx;          // Kernel row/col (0 .. kernel_size-1)
    reg [3:0]  wt_load_cnt;     // Weight sub-counter during CS_KER_WT (0..PE_ROWS-1)
    reg [3:0]  pe_row_sel;      // PE row selector during CS_POST_FEED (0..active_rows-1)
    reg [2:0]  drain_cnt;       // Drain cycle counter (0 .. POST_LATENCY-1)

    // ── Pool layer registers ───────────────────────────────────────────────────
    reg [8:0]  pool_ch;          // Current channel (0 .. in_channels-1)
    reg [15:0] pool_px;          // Pixel index within channel (0 .. img_dim²-1)
    reg [15:0] pool_px_total;    // img_dim * img_dim, latched at layer start
    reg        pool_layer_mode;  // 1 while running a pool layer
    reg        pool_fsync_r;     // Registered frame_sync pulse sent to pooling_unit
    reg [2:0]  pool_drain_cnt;   // Drain-cycle counter for pool pipeline

    // Number of active PE rows in the current oc_tile (last tile may be partial)
    wire [8:0] remaining_ch = out_channels - oc_base;
    wire [4:0] active_rows  = (remaining_ch >= 9'd16) ? 5'd16 : remaining_ch[4:0];

    // =========================================================================
    // Weight address tracking
    // wt_oc_start: base address of current oc_tile in weight buffer
    //              = oc_tile * in_ch * k * k * PE_ROWS
    // wt_ker_base: start of current (ic, ky, kx) block within tile
    //              reset to wt_oc_start at start of each new (oy, ox)
    //              incremented by PE_ROWS at each CS_KER_NEXT
    // ker_vol_per_tile: number of weight entries per oc_tile
    //              = in_channels * kernel_size * kernel_size * PE_ROWS
    // =========================================================================
    reg [WB_AW-1:0] wt_oc_start;
    reg [WB_AW-1:0] wt_ker_base;
    wire [WB_AW-1:0] ker_vol_per_tile;
    assign ker_vol_per_tile = {7'b0, in_channels} *
                              ({6'b0, kernel_size} * {6'b0, kernel_size}) *
                              16;  // PE_ROWS = 16

    // =========================================================================
    // Weight register bank: PE_ROWS entries, one per output channel in tile
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] weight_reg [0:PE_ROWS-1];

    // =========================================================================
    // ifmap register: holds the single input pixel for the current MAC
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] ifmap_reg;
    reg                         in_bounds_prev;  // Registered from CS_KER_REQ

    // =========================================================================
    // Combinatorial: current kernel input position and ifmap address
    // =========================================================================
    // iy = oy + ky - pad (signed, may be negative at edges)
    // ix = ox + kx - pad
    wire signed [8:0] iy_s;
    wire signed [8:0] ix_s;
    assign iy_s = $signed({1'b0, oy}) + $signed({1'b0, ky}) - (pad_en ? 9'sd1 : 9'sd0);
    assign ix_s = $signed({1'b0, ox}) + $signed({1'b0, kx}) - (pad_en ? 9'sd1 : 9'sd0);

    wire in_bounds_comb;
    assign in_bounds_comb = (iy_s >= 0) && (iy_s < $signed({1'b0, img_dim})) &&
                            (ix_s >= 0) && (ix_s < $signed({1'b0, img_dim}));

    // ifmap address: [ic][iy][ix] = ic * img_dim^2 + iy * img_dim + ix
    // All components are unsigned when in_bounds; result fits in BUF_AW_IFMAP=16 bits
    wire [15:0] ifmap_addr_comb;
    wire [15:0] img_dim_sq = {8'b0, img_dim} * {8'b0, img_dim};

    // Pool layer: sequential ifmap address = pool_ch * img_dim² + pool_px
    // Width 19 bits covers the maximum: 256 ch × 128×128 = 4,194,304 (22 bits) — but
    // BUF_AW_IFMAP determines actual addressable range; caller must size buffer appropriately.
    wire [21:0] pool_ifmap_addr_wide;
    wire [BUF_AW_IFMAP-1:0] pool_ifmap_addr;
    assign pool_ifmap_addr_wide = ({13'b0, pool_ch} * {6'b0, img_dim_sq}) + {6'b0, pool_px};
    assign pool_ifmap_addr      = pool_ifmap_addr_wide[BUF_AW_IFMAP-1:0];

    assign ifmap_addr_comb = ({7'b0, ic} * img_dim_sq) +
                             ($unsigned(iy_s[7:0]) * {8'b0, img_dim}) +
                             $unsigned(ix_s[7:0]);

    // =========================================================================
    // Memory read control (combinatorial)
    // =========================================================================

    // ifmap buffer read
    wire ifmap_rd_en;
    wire [BUF_AW_IFMAP-1:0] ifmap_rd_addr;
    wire signed [DATA_WIDTH-1:0] ifmap_rd_data;
    wire ifmap_rd_valid;

    assign ifmap_rd_en   = ((cs_state == CS_KER_REQ) && in_bounds_comb) ||
                           (cs_state == CS_POOL_RUN);
    // ifmap_addr_comb is 16-bit; assign directly to BUF_AW_IFMAP-bit wire —
    // Verilog zero-extends since CONV addresses always fit in 16 bits.
    assign ifmap_rd_addr = (cs_state == CS_POOL_RUN) ? pool_ifmap_addr :
                           ifmap_addr_comb;

    // Weight buffer read
    wire wt_rd_en;
    wire [WB_AW-1:0] wt_rd_addr;
    wire signed [DATA_WIDTH-1:0] wt_rd_data;
    wire wt_rd_valid;

    // Issue weight reads in CS_KER_REQ (weight[0]) and CS_KER_WT cnt=0..14 (weight[1..15])
    assign wt_rd_en   = (cs_state == CS_KER_REQ) ||
                        (cs_state == CS_KER_WT && wt_load_cnt < (PE_ROWS - 1));
    assign wt_rd_addr = (cs_state == CS_KER_REQ) ? wt_ker_base :
                        (cs_state == CS_KER_WT)   ? (wt_ker_base + {{(WB_AW-4){1'b0}}, wt_load_cnt} + 1) :
                        {WB_AW{1'b0}};

    // =========================================================================
    // PE array signals
    // =========================================================================

    // Weight bus: column 0 of each row = weight_reg[row]; all other columns = 0
    wire signed [PE_ROWS*PE_COLS*DATA_WIDTH-1:0] pe_weights_bus;
    genvar gr;
    generate
        for (gr = 0; gr < PE_ROWS; gr = gr + 1) begin : gen_wbus
            // Column 0: weight from register bank
            assign pe_weights_bus[(gr * PE_COLS + 0) * DATA_WIDTH +: DATA_WIDTH] = weight_reg[gr];
            // Columns 1..PE_COLS-1: zero
            assign pe_weights_bus[(gr * PE_COLS + 1) * DATA_WIDTH +: (PE_COLS-1)*DATA_WIDTH] =
                {(PE_COLS-1)*DATA_WIDTH{1'b0}};
        end
    endgenerate

    // ifmap bus: column 0 = ifmap_reg; columns 1-15 = 0
    wire signed [PE_COLS*DATA_WIDTH-1:0] pe_ifmap_bus;
    assign pe_ifmap_bus = {{(PE_COLS-1)*DATA_WIDTH{1'b0}}, ifmap_reg};

    // PE control
    wire pe_en;
    wire pe_clear;
    wire [PE_ROWS-1:0] pe_row_en;
    wire [PE_COLS-1:0] pe_col_en;
    wire signed [PE_ROWS*PE_COLS*ACC_WIDTH-1:0] pe_acc_out;

    assign pe_en     = (cs_state == CS_KER_MAC);
    assign pe_clear  = (cs_state == CS_CLEAR);
    assign pe_row_en = {PE_ROWS{1'b1}};
    assign pe_col_en = 16'h0001;  // Only column 0 active

    // =========================================================================
    // Post-processing mux: select PE row output and matching bias
    // =========================================================================
    localparam PE_ROW_STRIDE = PE_COLS * ACC_WIDTH;  // 512 bits per PE row in acc_out

    wire signed [ACC_WIDTH-1:0]  pe_acc_sel;
    wire signed [`BIAS_WIDTH-1:0] bias_val_sel;

    // Variable part-select: pe_row_sel * PE_ROW_STRIDE bits from pe_acc_out (column 0 of each row)
    assign pe_acc_sel   = pe_acc_out[pe_row_sel * PE_ROW_STRIDE +: ACC_WIDTH];
    assign bias_val_sel = bias_rom[oc_base + {{5{1'b0}}, pe_row_sel}];

    // Post-proc valid signal
    wire post_in_valid;
    assign post_in_valid = (cs_state == CS_POST_FEED);

    // =========================================================================
    // Post-processing pipeline (unchanged sub-modules)
    // =========================================================================
    wire signed [ACC_WIDTH-1:0]  bias_result;
    wire                         bias_valid;
    wire signed [DATA_WIDTH-1:0] rq_result;
    wire                         rq_valid;
    wire signed [DATA_WIDTH-1:0] act_result;
    wire                         act_valid;
    wire signed [DATA_WIDTH-1:0] pool_result;
    wire                         pool_valid;

    bias_add #(
        .ACC_WIDTH  (ACC_WIDTH),
        .BIAS_WIDTH (`BIAS_WIDTH)
    ) u_bias_add (
        .clk      (clk),
        .rst      (rst),
        .acc_in   (pe_acc_sel),
        .bias_in  (bias_val_sel),
        .in_valid (post_in_valid),
        .result   (bias_result),
        .out_valid(bias_valid)
    );

    requantizer #(
        .ACC_WIDTH   (ACC_WIDTH),
        .DATA_WIDTH  (DATA_WIDTH),
        .MULT_WIDTH  (`MULT_WIDTH),
        .SHIFT_WIDTH (`SHIFT_WIDTH)
    ) u_requantizer (
        .clk        (clk),
        .rst        (rst),
        .data_in    (bias_result),
        .in_valid   (bias_valid),
        .multiplier (rq_multiplier),
        .shift      (rq_shift),
        .zero_point (rq_zero_point),
        .data_out   (rq_result),
        .out_valid  (rq_valid)
    );

    activation_unit #(
        .DATA_WIDTH (DATA_WIDTH),
        .LEAK_SHIFT (`LEAK_SHIFT)
    ) u_activation (
        .clk      (clk),
        .rst      (rst),
        .data_in  (rq_result),
        .in_valid (rq_valid),
        .mode     (act_mode),
        .data_out (act_result),
        .out_valid(act_valid)
    );

    // ── Pool-layer data mux ───────────────────────────────────────────────────
    // During a pool layer: bypass the PE/post-proc pipeline and feed ifmap
    // directly into pooling_unit.  During a conv layer: use the normal path
    // (act_result from activation_unit).  The 'bypass' port of pooling_unit
    // stays tied to ~pool_en so it always reflects the layer configuration.
    wire signed [DATA_WIDTH-1:0] pool_data_in_mux;
    wire                         pool_in_valid_mux;
    assign pool_data_in_mux  = pool_layer_mode ? ifmap_rd_data : act_result;
    assign pool_in_valid_mux = pool_layer_mode ? ifmap_rd_valid : act_valid;

    pooling_unit #(
        .DATA_WIDTH  (DATA_WIDTH),
        .MAX_IMG_DIM (MAX_DIM)
    ) u_pooling (
        .clk        (clk),
        .rst        (rst),
        .row_size   (img_dim),
        .bypass     (~pool_en),
        .frame_sync (pool_layer_mode ? pool_fsync_r : compute_start),
        .data_in    (pool_data_in_mux),
        .in_valid   (pool_in_valid_mux),
        .data_out   (pool_result),
        .out_valid  (pool_valid)
    );

    // =========================================================================
    // ifmap buffer (BUF_DEPTH_IFMAP = 65536)
    // =========================================================================
    ifmap_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (BUF_DEPTH_IFMAP)
    ) u_ifmap_buf (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (ifmap_ext_wr_en),
        .wr_addr (ifmap_ext_wr_addr),
        .wr_data (ifmap_ext_wr_data),
        .rd_en   (ifmap_rd_en),
        .rd_addr (ifmap_rd_addr),
        .rd_data (ifmap_rd_data),
        .rd_valid(ifmap_rd_valid)
    );

    // =========================================================================
    // Weight buffer (BUF_DEPTH = 65536, tiled-format weights)
    // =========================================================================
    weight_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (WB_DEPTH)
    ) u_weight_buf (
        .clk     (clk),
        .rst     (rst),
        .ld_en   (wt_ld_en),
        .ld_addr (wt_ld_addr),
        .ld_data (wt_ld_data),
        .rd_en   (wt_rd_en),
        .rd_addr (wt_rd_addr),
        .rd_data (wt_rd_data),
        .rd_valid(wt_rd_valid),
        .swap    (wt_swap)
    );

    // =========================================================================
    // ofmap buffer (BUF_DEPTH_OFMAP = 262144, sequential write from stream)
    // Sequential write address increments with each pool_valid; the stream order
    // is [oc_tile][oy][ox][row] which equals [oy][ox][oc] for a single tile.
    // =========================================================================
    reg [BUF_AW_OFMAP-1:0] ofmap_wr_idx;

    always @(posedge clk) begin
        if (rst || compute_start)
            ofmap_wr_idx <= 0;
        else if (pool_valid)
            ofmap_wr_idx <= ofmap_wr_idx + 1;
    end

    ofmap_buffer #(
        .DATA_WIDTH (DATA_WIDTH),
        .BUF_DEPTH  (BUF_DEPTH_OFMAP)
    ) u_ofmap_buf (
        .clk     (clk),
        .rst     (rst),
        .wr_en   (pool_valid),
        .wr_addr (ofmap_wr_idx),
        .wr_data (pool_result),
        .rd_en   (1'b0),
        .rd_addr ({BUF_AW_OFMAP{1'b0}}),
        .rd_data (),
        .rd_valid()
    );

    // =========================================================================
    // PE array
    // =========================================================================
    pe_array #(
        .PE_ROWS    (PE_ROWS),
        .PE_COLS    (PE_COLS),
        .DATA_WIDTH (DATA_WIDTH),
        .ACC_WIDTH  (ACC_WIDTH)
    ) u_pe_array (
        .clk        (clk),
        .rst        (rst),
        .en         (pe_en),
        .acc_clear  (pe_clear),
        .row_en     (pe_row_en),
        .col_en     (pe_col_en),
        .ifmap_data (pe_ifmap_bus),
        .weight_data(pe_weights_bus),
        .acc_out    (pe_acc_out)
    );

    // =========================================================================
    // Compute FSM
    // =========================================================================
    integer i_init;  // Loop variable for weight_reg initialization at reset

    always @(posedge clk) begin
        if (rst) begin
            cs_state       <= CS_IDLE;
            compute_done   <= 1'b0;
            oc_tile        <= 0;
            oc_tile_max    <= 0;
            oc_base        <= 0;
            oy             <= 0;
            ox             <= 0;
            ic             <= 0;
            ky             <= 0;
            kx             <= 0;
            wt_load_cnt    <= 0;
            pe_row_sel     <= 0;
            drain_cnt      <= 0;
            wt_oc_start    <= 0;
            wt_ker_base    <= 0;
            in_bounds_prev  <= 1'b0;
            ifmap_reg       <= 0;
            pool_ch         <= 0;
            pool_px         <= 0;
            pool_px_total   <= 0;
            pool_layer_mode <= 1'b0;
            pool_fsync_r    <= 1'b0;
            pool_drain_cnt  <= 0;
            // Initialize weight registers to zero
            for (i_init = 0; i_init < PE_ROWS; i_init = i_init + 1)
                weight_reg[i_init] <= 0;
        end else begin
            compute_done <= 1'b0;  // Default: deassert

            case (cs_state)

                // -------------------------------------------------------
                CS_IDLE: begin
                    if (compute_start && layer_type == `LAYER_TYPE_CONV) begin
                        oc_tile     <= 0;
                        oc_base     <= 0;
                        oc_tile_max <= (out_channels + (PE_ROWS - 1)) >> $clog2(PE_ROWS);
                        oy          <= 0;
                        ox          <= 0;
                        wt_oc_start <= 0;
                        wt_ker_base <= 0;
                        cs_state    <= CS_CLEAR;
                    end else if (compute_start && layer_type == `LAYER_TYPE_POOL) begin
                        // Pool layers: sequential read from ifmap_buffer into pooling_unit.
                        // Note: stride=0 (stride-1 darknet, L11 only) is not yet supported;
                        // only stride=1 (stride-2) is handled.  L11 falls through with 0 outputs.
                        pool_ch         <= 0;
                        pool_px         <= 0;
                        pool_px_total   <= img_dim_sq;
                        pool_layer_mode <= 1'b1;
                        cs_state        <= CS_POOL_SYNC;
                    end
                end

                // -------------------------------------------------------
                // Assert pe_clear for 1 cycle; reset kernel counters and
                // restore wt_ker_base to the start of the current oc_tile.
                // -------------------------------------------------------
                CS_CLEAR: begin
                    ic          <= 0;
                    ky          <= 0;
                    kx          <= 0;
                    wt_load_cnt <= 0;
                    wt_ker_base <= wt_oc_start;  // Restart from tile base each new (oy, ox)
                    cs_state    <= CS_KER_REQ;
                end

                // -------------------------------------------------------
                // Cycle: issue ifmap read (if in_bounds) + weight[0] read.
                // Data will be available next cycle (CS_KER_WT cnt=0).
                // -------------------------------------------------------
                CS_KER_REQ: begin
                    in_bounds_prev <= in_bounds_comb;
                    wt_load_cnt    <= 0;
                    cs_state       <= CS_KER_WT;
                end

                // -------------------------------------------------------
                // Cycles 0..PE_ROWS-1: latch weight[n] from weight buffer.
                // At cnt=0 also latch ifmap (or zero if out-of-bounds).
                // Issue next weight read for cnt < PE_ROWS-1.
                // -------------------------------------------------------
                CS_KER_WT: begin
                    // Latch weight data from weight buffer (1-cycle read latency)
                    weight_reg[wt_load_cnt] <= wt_rd_data;

                    // Latch ifmap only on first cycle; use 0 for padding positions
                    if (wt_load_cnt == 0)
                        ifmap_reg <= in_bounds_prev ? ifmap_rd_data : {DATA_WIDTH{1'b0}};

                    if (wt_load_cnt == PE_ROWS - 1) begin
                        // All weights loaded; proceed to MAC
                        cs_state <= CS_KER_MAC;
                    end else begin
                        wt_load_cnt <= wt_load_cnt + 1;
                    end
                end

                // -------------------------------------------------------
                // Single MAC cycle: pe_en=1, pe_col_en=16'h0001.
                // PE[r][0].acc += ifmap_reg * weight_reg[r]  for all r.
                // -------------------------------------------------------
                CS_KER_MAC: begin
                    cs_state <= CS_KER_NEXT;
                end

                // -------------------------------------------------------
                // Advance kernel position (kx → ky → ic).
                // Update wt_ker_base for the next kernel block.
                // -------------------------------------------------------
                CS_KER_NEXT: begin
                    wt_ker_base <= wt_ker_base + PE_ROWS;

                    if (kx < kernel_size - 1) begin
                        kx       <= kx + 1;
                        cs_state <= CS_KER_REQ;
                    end else begin
                        kx <= 0;
                        if (ky < kernel_size - 1) begin
                            ky       <= ky + 1;
                            cs_state <= CS_KER_REQ;
                        end else begin
                            ky <= 0;
                            if (ic < in_channels - 1) begin
                                ic       <= ic + 1;
                                cs_state <= CS_KER_REQ;
                            end else begin
                                // Kernel scan complete for (oc_tile, oy, ox)
                                ic         <= 0;
                                pe_row_sel <= 0;
                                cs_state   <= CS_POST_FEED;
                            end
                        end
                    end
                end

                // -------------------------------------------------------
                // Feed each PE row's accumulator result through post-proc.
                // One result per cycle: bias_add → rq → act → pool.
                // -------------------------------------------------------
                CS_POST_FEED: begin
                    if (pe_row_sel < active_rows - 1) begin
                        pe_row_sel <= pe_row_sel + 1;
                    end else begin
                        drain_cnt <= 0;
                        cs_state  <= CS_POST_DRAIN;
                    end
                end

                // -------------------------------------------------------
                // Wait POST_LATENCY cycles for the last output to emerge.
                // -------------------------------------------------------
                CS_POST_DRAIN: begin
                    if (drain_cnt == POST_LATENCY - 1) begin
                        cs_state <= CS_SPATIAL_ADV;
                    end else begin
                        drain_cnt <= drain_cnt + 1;
                    end
                end

                // -------------------------------------------------------
                // Advance ox/oy spatial counters.
                // -------------------------------------------------------
                CS_SPATIAL_ADV: begin
                    if (ox < img_dim - 1) begin
                        ox       <= ox + 1;
                        cs_state <= CS_CLEAR;
                    end else begin
                        ox <= 0;
                        if (oy < img_dim - 1) begin
                            oy       <= oy + 1;
                            cs_state <= CS_CLEAR;
                        end else begin
                            // All spatial positions done for this oc_tile
                            oy       <= 0;
                            cs_state <= CS_OC_ADV;
                        end
                    end
                end

                // -------------------------------------------------------
                // Advance oc_tile; compute new wt_oc_start.
                // -------------------------------------------------------
                CS_OC_ADV: begin
                    if (oc_tile < oc_tile_max - 1) begin
                        oc_tile     <= oc_tile + 1;
                        oc_base     <= oc_base + PE_ROWS;
                        wt_oc_start <= wt_oc_start + ker_vol_per_tile;
                        cs_state    <= CS_CLEAR;
                    end else begin
                        cs_state <= CS_DONE;
                    end
                end

                // -------------------------------------------------------
                CS_DONE: begin
                    compute_done <= 1'b1;
                    cs_state     <= CS_IDLE;
                end

                // =========================================================
                // Pool layer FSM (CS_POOL_SYNC → CS_POOL_RUN → CS_POOL_DRAIN
                //                 → CS_POOL_NEXT → CS_POOL_SYNC or CS_DONE)
                //
                // Each channel is processed independently:
                //   1. CS_POOL_SYNC:  Assert pool_fsync_r=1 (resets pooling_unit
                //                    counters), reset pixel counter.
                //   2. CS_POOL_RUN:   On the first cycle pool_fsync_r is still 1
                //                    (registered from SYNC), so pooling_unit resets
                //                    while ifmap read for px=0 is issued.
                //                    Subsequent cycles: pool_fsync_r=0; each cycle
                //                    issues a read; ifmap_rd_valid feeds pooling_unit
                //                    one cycle later (1-cycle read latency).
                //   3. CS_POOL_DRAIN: Drain 4 extra cycles to flush the last pool
                //                    output through pooling_unit's registered path.
                //   4. CS_POOL_NEXT:  Advance channel; loop back to SYNC or go to DONE.
                //
                // Limitations: stride=0 (stride-1 darknet, L11 only) falls back to
                // an identity pass (no-op); correct L11 is deferred.
                // =========================================================

                CS_POOL_SYNC: begin
                    pool_fsync_r    <= 1'b1;  // Will be seen by pooling_unit on first cycle of RUN
                    pool_px         <= 0;
                    pool_drain_cnt  <= 0;
                    cs_state        <= CS_POOL_RUN;
                end

                CS_POOL_RUN: begin
                    pool_fsync_r <= 1'b0;  // Deassert after first cycle
                    if (pool_px < pool_px_total - 1) begin
                        pool_px  <= pool_px + 1;
                    end else begin
                        // Last read issued; enter drain
                        cs_state <= CS_POOL_DRAIN;
                    end
                end

                CS_POOL_DRAIN: begin
                    // Wait 4 cycles: 1 for last ifmap read, 1 for pooling_unit to
                    // register out_valid, 2 for margin.
                    if (pool_drain_cnt == 3) begin
                        cs_state <= CS_POOL_NEXT;
                    end else begin
                        pool_drain_cnt <= pool_drain_cnt + 1;
                    end
                end

                CS_POOL_NEXT: begin
                    if (pool_ch < in_channels - 1) begin
                        pool_ch  <= pool_ch + 1;
                        cs_state <= CS_POOL_SYNC;
                    end else begin
                        pool_layer_mode <= 1'b0;
                        cs_state        <= CS_DONE;
                    end
                end

                default: cs_state <= CS_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Output interface: stream the post-processing results
    // =========================================================================
    assign out_data  = pool_result;
    assign out_valid = pool_valid;
    assign out_last  = (cs_state == CS_DONE);

endmodule
