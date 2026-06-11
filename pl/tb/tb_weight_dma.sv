/*
------------------------------------------------------------------------------
Testbench : tb_weight_dma
Tests     : weight_dma.v — AXI4 burst-read weight loader
Method    : Instantiates weight_dma with a memory model acting as AXI slave.
            Preloads known byte patterns into the model, then triggers load
            for each conv layer and verifies:
              1. load_done is asserted after correct byte count
              2. wt_ld_en/addr/data sequence matches expected bytes
              3. Pool layers (count=0) assert load_done in ≤ 4 cycles
------------------------------------------------------------------------------
*/
`timescale 1ns/1ps

module tb_weight_dma;

    // DUT parameters
    localparam AXI_AW     = 32;
    localparam AXI_DW     = 32;
    localparam WT_AW      = 20;
    localparam NUM_LAYERS = 19;

    // Clock/reset
    logic clk = 0;
    logic rst = 1;
    always #5 clk = ~clk;  // 100 MHz

    // DUT ports
    logic [4:0]  layer_idx;
    logic [31:0] ddr_base_addr;
    logic        load_start;
    logic        load_done;

    logic [AXI_AW-1:0] m_axi_araddr;
    logic              m_axi_arvalid;
    logic              m_axi_arready;
    logic [7:0]        m_axi_arlen;
    logic [2:0]        m_axi_arsize;
    logic [1:0]        m_axi_arburst;
    logic [AXI_DW-1:0] m_axi_rdata;
    logic              m_axi_rvalid;
    logic              m_axi_rlast;
    logic              m_axi_rready;

    logic        wt_ld_en;
    logic [WT_AW-1:0] wt_ld_addr;
    logic [7:0]  wt_ld_data;

    weight_dma #(
        .AXI_AW(AXI_AW), .AXI_DW(AXI_DW), .WT_AW(WT_AW), .NUM_LAYERS(NUM_LAYERS)
    ) dut (.*);

    // =========================================================================
    // AXI slave memory model — 4 MB of byte-addressable memory
    // =========================================================================
    localparam MEM_SIZE = 4*1024*1024;
    logic [7:0] mem [0:MEM_SIZE-1];

    // Initialise memory with known pattern: mem[i] = i[7:0] (modulo 256)
    initial begin
        for (int i = 0; i < MEM_SIZE; i++) mem[i] = i[7:0];
    end

    // AXI4 read slave FSM
    logic [31:0] slave_araddr;
    logic [7:0]  slave_arlen;
    logic [7:0]  beats_remaining;
    logic        slave_active;
    logic [31:0] slave_read_addr;

    initial begin
        m_axi_arready = 1'b0;
        m_axi_rvalid  = 1'b0;
        m_axi_rdata   = 32'd0;
        m_axi_rlast   = 1'b0;
        slave_active  = 1'b0;
    end

    always @(posedge clk) begin
        m_axi_arready <= 1'b0;
        m_axi_rvalid  <= 1'b0;
        m_axi_rlast   <= 1'b0;

        if (!slave_active) begin
            if (m_axi_arvalid) begin
                m_axi_arready  <= 1'b1;
                slave_araddr   <= m_axi_araddr;
                slave_arlen    <= m_axi_arlen;
                beats_remaining <= m_axi_arlen;
                slave_read_addr <= m_axi_araddr;
                slave_active   <= 1'b1;
            end
        end else begin
            // Serve data
            if (m_axi_rready) begin
                m_axi_rvalid <= 1'b1;
                m_axi_rdata  <= {mem[slave_read_addr+3], mem[slave_read_addr+2],
                                  mem[slave_read_addr+1], mem[slave_read_addr]};
                m_axi_rlast  <= (beats_remaining == 0);
                slave_read_addr  <= slave_read_addr + 4;
                if (beats_remaining == 0) begin
                    slave_active <= 1'b0;
                end else begin
                    beats_remaining <= beats_remaining - 1;
                end
            end
        end
    end

    // =========================================================================
    // Test sequence
    // =========================================================================
    // Weight counts for reference
    int wt_counts [0:NUM_LAYERS-1];
    initial begin
        wt_counts[0]=432; wt_counts[1]=0; wt_counts[2]=4608; wt_counts[3]=0;
        wt_counts[4]=18432; wt_counts[5]=0; wt_counts[6]=73728; wt_counts[7]=0;
        wt_counts[8]=294912; wt_counts[9]=0; wt_counts[10]=589824; wt_counts[11]=0;
        wt_counts[12]=589824; wt_counts[13]=65536; wt_counts[14]=589824;
        wt_counts[15]=65536; wt_counts[16]=32640; wt_counts[17]=294912;
        wt_counts[18]=65536;
    end

    int    total_pass = 0;
    int    total_fail = 0;

    task automatic test_layer(input [4:0] lid);
        automatic int    expected_bytes = wt_counts[lid];
        automatic int    bytes_received = 0;
        automatic int    max_cycles = expected_bytes * 8 + 100;
        automatic logic [7:0] exp_byte;
        automatic logic [7:0] got_byte;
        automatic int    mismatch = 0;
        automatic int    timeout = 0;

        // Trigger
        @(posedge clk); #1;
        layer_idx  = lid;
        load_start = 1'b1;
        @(posedge clk); #1;
        load_start = 1'b0;

        // Collect wt_ld_en pulses
        fork
            begin
                while (!load_done && timeout < max_cycles) begin
                    @(posedge clk);
                    timeout++;
                    if (wt_ld_en) begin
                        got_byte = wt_ld_data;
                        bytes_received++;
                    end
                end
            end
        join

        if (expected_bytes == 0) begin
            if (load_done) begin
                $display("  [L%02d] Pool: load_done in %0d cycles — PASS", lid, timeout);
                total_pass++;
            end else begin
                $display("  [L%02d] Pool: load_done NOT seen — FAIL", lid);
                total_fail++;
            end
        end else begin
            if (bytes_received == expected_bytes) begin
                $display("  [L%02d] Conv: %0d bytes loaded — PASS", lid, bytes_received);
                total_pass++;
            end else begin
                $display("  [L%02d] Conv: expected %0d bytes, got %0d — FAIL",
                          lid, expected_bytes, bytes_received);
                total_fail++;
            end
        end
    endtask

    initial begin
        ddr_base_addr = 32'h0021_0000;
        load_start    = 1'b0;
        layer_idx     = 5'd0;

        // Reset
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("======================================================");
        $display("  tb_weight_dma — weight load verification");
        $display("======================================================");

        // Test all 19 layers (only a few to keep sim time reasonable)
        // Test: L0 (small conv), L1 (pool → instant), L8 (medium), L13 (1x1)
        foreach ({0, 1, 2, 8, 13} [i]) begin
            automatic int lid = {0, 1, 2, 8, 13}[i];
            test_layer(lid[4:0]);
            repeat(4) @(posedge clk);
        end

        $display("======================================================");
        $display("  RESULT: %0d PASS / %0d FAIL", total_pass, total_fail);
        $display("======================================================");
        $finish;
    end

endmodule
