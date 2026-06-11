/*
------------------------------------------------------------------------------
Module Name : weight_dma
Project     : EdgeAI Location Intelligence — Zynq Integration
Description :
    AXI4 burst-read master that loads one layer's INT8 weights from PS DDR3
    into cnn_core's weight_buffer via wt_ld_en/wt_ld_addr/wt_ld_data.

    Called once per CONV layer by layer_scheduler (via load_start).
    Pool layers have weight_count=0 and return load_done immediately.

    Architecture:
    - Layer weight count and DDR offset are stored in internal ROMs.
    - AXI4 burst reads (INCR, 4-byte, up to 256 beats per burst).
    - 32-bit words from AXI are decomposed into 4 consecutive byte writes
      to weight_buffer (LSB first, matching DDR little-endian layout).
    - At 100 MHz: largest layer (L10, 589824 bytes) loads in ~5.9M cycles = ~59 ms.

    Layer weight byte counts (tiled INT8 format required by cnn_core):
      L0:432  L2:4608  L4:18432  L6:73728  L8:294912
      L10-L12-L14: 589824 each
      L13,L15,L18: 65536  L16:32640  L17:294912

    Cumulative DDR offsets from ddr_base_addr (bytes):
      L0:0  L2:432  L4:5040  L6:23472  L8:97200  L10:392112
      L12:981936  L13:1571760  L14:1637296  L15:2227120
      L16:2292656  L17:2325296  L18:2620208
      (Total blob = 2,685,744 bytes ≈ 2.56 MB)
------------------------------------------------------------------------------
*/

module weight_dma #(
    parameter AXI_AW     = 32,
    parameter AXI_DW     = 32,   // 32-bit AXI4 to HP port
    parameter WT_AW      = 20,   // Must match cnn_core WB_AW parameter
    parameter NUM_LAYERS = 19
)(
    input  wire        clk,
    input  wire        rst,

    // Control
    input  wire [4:0]  layer_idx,       // 0..18
    input  wire [31:0] ddr_base_addr,   // Base of weight blob in PS DDR3
    input  wire        load_start,      // 1-cycle pulse to begin
    output reg         load_done,       // 1-cycle pulse when complete

    // AXI4 Read Address Channel
    output reg  [AXI_AW-1:0] m_axi_araddr,
    output reg               m_axi_arvalid,
    input  wire              m_axi_arready,
    output reg  [7:0]        m_axi_arlen,    // burst length - 1
    output wire [2:0]        m_axi_arsize,   // 4-byte
    output wire [1:0]        m_axi_arburst,  // INCR

    // AXI4 Read Data Channel
    input  wire [AXI_DW-1:0] m_axi_rdata,
    input  wire              m_axi_rvalid,
    input  wire              m_axi_rlast,
    output reg               m_axi_rready,

    // weight_buffer write port
    output reg               wt_ld_en,
    output reg  [WT_AW-1:0] wt_ld_addr,
    output reg  [7:0]        wt_ld_data
);

    assign m_axi_arsize  = 3'd2;  // 4 bytes
    assign m_axi_arburst = 2'd1;  // INCR

    // =========================================================================
    // Layer ROM: weight byte counts and DDR offsets
    // =========================================================================
    reg [19:0] wt_count  [0:NUM_LAYERS-1];
    reg [21:0] wt_offset [0:NUM_LAYERS-1];

    initial begin
        wt_count[0]=20'd432;     wt_offset[0]=22'd0;
        wt_count[1]=20'd0;       wt_offset[1]=22'd432;
        wt_count[2]=20'd4608;    wt_offset[2]=22'd432;
        wt_count[3]=20'd0;       wt_offset[3]=22'd5040;
        wt_count[4]=20'd18432;   wt_offset[4]=22'd5040;
        wt_count[5]=20'd0;       wt_offset[5]=22'd23472;
        wt_count[6]=20'd73728;   wt_offset[6]=22'd23472;
        wt_count[7]=20'd0;       wt_offset[7]=22'd97200;
        wt_count[8]=20'd294912;  wt_offset[8]=22'd97200;
        wt_count[9]=20'd0;       wt_offset[9]=22'd392112;
        wt_count[10]=20'd589824; wt_offset[10]=22'd392112;
        wt_count[11]=20'd0;      wt_offset[11]=22'd981936;
        wt_count[12]=20'd589824; wt_offset[12]=22'd981936;
        wt_count[13]=20'd65536;  wt_offset[13]=22'd1571760;
        wt_count[14]=20'd589824; wt_offset[14]=22'd1637296;
        wt_count[15]=20'd65536;  wt_offset[15]=22'd2227120;
        wt_count[16]=20'd32640;  wt_offset[16]=22'd2292656;
        wt_count[17]=20'd294912; wt_offset[17]=22'd2325296;
        wt_count[18]=20'd65536;  wt_offset[18]=22'd2620208;
    end

    // =========================================================================
    // FSM
    // =========================================================================
    localparam ST_IDLE      = 3'd0;
    localparam ST_LATCH     = 3'd1;
    localparam ST_AR        = 3'd2;   // Issue read address
    localparam ST_RDATA     = 3'd3;   // Receive data beat
    localparam ST_BYTES     = 3'd4;   // Write 4 bytes from latched word
    localparam ST_CHK       = 3'd5;   // Decide: next burst or done
    localparam ST_DONE      = 3'd6;

    reg [2:0]  state;

    reg [21:0] total_bytes;
    reg [21:0] bytes_done;
    reg [31:0] cur_ddr_addr;
    reg [WT_AW-1:0] wt_idx;

    reg [31:0] word_latch;   // current 32-bit word being unpacked
    reg [1:0]  byte_ptr;     // 0..3: which byte of word_latch to write next
    reg        last_beat;    // rlast was seen

    // Compute burst length for next burst (max 256 beats = 1KB)
    wire [21:0] words_remaining = (total_bytes - bytes_done + 3) >> 2;
    wire [7:0]  next_arlen = (words_remaining >= 22'd256) ? 8'd255 :
                              words_remaining[7:0] - 8'd1;

    always @(posedge clk) begin
        if (rst) begin
            state         <= ST_IDLE;
            load_done     <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_rready  <= 1'b0;
            wt_ld_en      <= 1'b0;
            wt_idx        <= 0;
            bytes_done    <= 0;
            byte_ptr      <= 0;
            last_beat     <= 1'b0;
        end else begin
            // Defaults
            load_done  <= 1'b0;
            wt_ld_en   <= 1'b0;

            case (state)

                // ----------------------------------------------------------
                ST_IDLE: begin
                    if (load_start) state <= ST_LATCH;
                end

                // ----------------------------------------------------------
                ST_LATCH: begin
                    total_bytes  <= {2'b0, wt_count[layer_idx]};
                    cur_ddr_addr <= ddr_base_addr + {10'd0, wt_offset[layer_idx]};
                    bytes_done   <= 0;
                    wt_idx       <= 0;
                    last_beat    <= 1'b0;
                    if (wt_count[layer_idx] == 0)
                        state <= ST_DONE;  // Pool layer — instant done
                    else
                        state <= ST_AR;
                end

                // ----------------------------------------------------------
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

                // ----------------------------------------------------------
                // Accept one beat; immediately enter byte-unpack state.
                // m_axi_rready stays asserted while we unpack so next beat
                // can be accepted as soon as byte_ptr reaches 3.
                ST_RDATA: begin
                    m_axi_rready <= 1'b1;
                    if (m_axi_rvalid) begin
                        word_latch <= m_axi_rdata;
                        byte_ptr   <= 2'd0;
                        last_beat  <= m_axi_rlast;
                        m_axi_rready <= 1'b0;  // Pause while we write 4 bytes
                        state <= ST_BYTES;
                    end
                end

                // ----------------------------------------------------------
                // Write up to 4 bytes from word_latch to weight_buffer.
                // One byte per cycle. After byte 3, return to ST_RDATA or CHK.
                ST_BYTES: begin
                    if (bytes_done < total_bytes) begin
                        wt_ld_en   <= 1'b1;
                        wt_ld_addr <= wt_idx;
                        case (byte_ptr)
                            2'd0: wt_ld_data <= word_latch[7:0];
                            2'd1: wt_ld_data <= word_latch[15:8];
                            2'd2: wt_ld_data <= word_latch[23:16];
                            2'd3: wt_ld_data <= word_latch[31:24];
                        endcase
                        wt_idx     <= wt_idx + 1;
                        bytes_done <= bytes_done + 1;
                    end

                    if (byte_ptr == 2'd3) begin
                        cur_ddr_addr <= cur_ddr_addr + 4;
                        if (last_beat)
                            state <= ST_CHK;
                        else begin
                            m_axi_rready <= 1'b1;
                            state <= ST_RDATA;
                        end
                    end else begin
                        byte_ptr <= byte_ptr + 1;
                    end
                end

                // ----------------------------------------------------------
                ST_CHK: begin
                    if (bytes_done >= total_bytes)
                        state <= ST_DONE;
                    else
                        state <= ST_AR;  // Next burst
                end

                // ----------------------------------------------------------
                ST_DONE: begin
                    load_done <= 1'b1;
                    state     <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
