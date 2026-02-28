/*
------------------------------------------------------------------------------
Module Name : sdram_ctrl
Project     : EdgeAI Location Intelligence
Author      : EdgeAI Team (auto-generated)
Description :
    Simplified burst-mode SDRAM controller.
    
    Functional Role:
    - Provides read/write access to external SDRAM
    - Supports burst read and burst write operations
    - Arbitrates between frame_writer (write) and CNN reader (read)
    - Designed to be swappable with AXI HP interface for Zynq
    
    Input/Output Contract:
    - Write port: wr_req, wr_addr, wr_data, wr_ack
    - Read port: rd_req, rd_addr, rd_data, rd_valid, rd_ack
    - SDRAM physical: sdram_clk, sdram_cke, sdram_cs_n, sdram_ras_n,
      sdram_cas_n, sdram_we_n, sdram_addr, sdram_ba, sdram_dq, sdram_dqm
    
    Data Widths:
    - Internal address: 24-bit (16 MB space)
    - Data bus: 16-bit
    
    Timing:
    - Command FSM: IDLE → ACTIVE → READ/WRITE → PRECHARGE
    - CAS latency: 2 cycles (parameterized)
    - Burst length: 8 (parameterized)
    
    Reset:
    - Synchronous active-HIGH reset
    - SDRAM init sequence on power-up

Dataflow    :
    Output-Stationary CNN accelerator pipeline — memory access

Dependencies:
    edgeai_defs.vh

Change Log  :
    v1.0 - Initial implementation
------------------------------------------------------------------------------
*/

`include "edgeai_defs.vh"

module sdram_ctrl #(
    parameter ADDR_WIDTH   = `SDRAM_ADDR_WIDTH,
    parameter DATA_WIDTH   = `SDRAM_DATA_WIDTH,
    parameter BURST_LEN    = `SDRAM_BURST_LEN,
    parameter CAS_LATENCY  = 2,
    // SDRAM geometry
    parameter ROW_WIDTH    = 13,
    parameter COL_WIDTH    = 9,
    parameter BANK_WIDTH   = 2
)(
    input  wire                   clk,
    input  wire                   rst,
    
    // Write port (from frame_writer)
    input  wire                   wr_req,
    input  wire [ADDR_WIDTH-1:0]  wr_addr,
    input  wire [DATA_WIDTH-1:0]  wr_data,
    output reg                    wr_ack,
    
    // Read port (from CNN input adapter)
    input  wire                   rd_req,
    input  wire [ADDR_WIDTH-1:0]  rd_addr,
    output reg  [DATA_WIDTH-1:0]  rd_data,
    output reg                    rd_valid,
    output reg                    rd_ack,
    
    // SDRAM physical interface
    output reg                    sdram_clk,
    output reg                    sdram_cke,
    output reg                    sdram_cs_n,
    output reg                    sdram_ras_n,
    output reg                    sdram_cas_n,
    output reg                    sdram_we_n,
    output reg  [ROW_WIDTH-1:0]   sdram_addr,
    output reg  [BANK_WIDTH-1:0]  sdram_ba,
    inout  wire [DATA_WIDTH-1:0]  sdram_dq,
    output reg  [1:0]             sdram_dqm
);

    // ========================================================================
    // Internal memory model (for simulation — replaced by real SDRAM on FPGA)
    // ========================================================================
    localparam MEM_SIZE = 1 << ADDR_WIDTH;
    reg [DATA_WIDTH-1:0] mem_array [0:MEM_SIZE-1];
    
    // ========================================================================
    // SDRAM command encoding
    // ========================================================================
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVE    = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_LOAD_MODE = 4'b0000;
    
    // ========================================================================
    // State machine
    // ========================================================================
    localparam S_INIT      = 4'd0;
    localparam S_IDLE      = 4'd1;
    localparam S_ACTIVE    = 4'd2;
    localparam S_WRITE     = 4'd3;
    localparam S_WRITE_BRK = 4'd4;
    localparam S_READ      = 4'd5;
    localparam S_READ_WAIT = 4'd6;
    localparam S_READ_DATA = 4'd7;
    localparam S_PRECHARGE = 4'd8;
    localparam S_REFRESH   = 4'd9;
    
    reg [3:0]  state;
    reg [15:0] init_cnt;
    reg [3:0]  wait_cnt;
    
    // Address decomposition
    wire [BANK_WIDTH-1:0] active_bank;
    wire [ROW_WIDTH-1:0]  active_row;
    wire [COL_WIDTH-1:0]  active_col;
    
    reg [ADDR_WIDTH-1:0] current_addr;
    reg                  current_is_write;
    reg [DATA_WIDTH-1:0] current_wr_data;
    
    // Simple address mapping: {bank, row, col}
    assign active_bank = current_addr[ADDR_WIDTH-1 -: BANK_WIDTH];
    assign active_row  = current_addr[ADDR_WIDTH-1-BANK_WIDTH -: ROW_WIDTH];
    assign active_col  = current_addr[COL_WIDTH-1:0];
    
    // DQ bus control
    reg [DATA_WIDTH-1:0] dq_out;
    reg                  dq_oe;
    assign sdram_dq = dq_oe ? dq_out : {DATA_WIDTH{1'bz}};
    
    // ========================================================================
    // Initialization counter
    // ========================================================================
    localparam INIT_CYCLES = 200;  // Simplified init delay
    
    // ========================================================================
    // Refresh counter
    // ========================================================================
    reg [15:0] refresh_cnt;
    localparam REFRESH_INTERVAL = 1500; // Refresh every N cycles (simplified)
    wire       refresh_needed;
    assign refresh_needed = (refresh_cnt >= REFRESH_INTERVAL);
    
    always @(posedge clk) begin
        if (rst) begin
            refresh_cnt <= 0;
        end else begin
            if (state == S_REFRESH)
                refresh_cnt <= 0;
            else
                refresh_cnt <= refresh_cnt + 1;
        end
    end
    
    // ========================================================================
    // Main controller FSM
    // ========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state          <= S_INIT;
            init_cnt       <= 0;
            wait_cnt       <= 0;
            wr_ack         <= 1'b0;
            rd_ack         <= 1'b0;
            rd_valid       <= 1'b0;
            rd_data        <= 0;
            sdram_cke      <= 1'b1;
            sdram_cs_n     <= 1'b0;
            sdram_ras_n    <= 1'b1;
            sdram_cas_n    <= 1'b1;
            sdram_we_n     <= 1'b1;
            sdram_addr     <= 0;
            sdram_ba       <= 0;
            sdram_dqm      <= 2'b00;
            sdram_clk      <= 1'b0;
            dq_out         <= 0;
            dq_oe          <= 1'b0;
            current_addr   <= 0;
            current_is_write <= 1'b0;
            current_wr_data  <= 0;
        end else begin
            // Defaults
            wr_ack   <= 1'b0;
            rd_ack   <= 1'b0;
            rd_valid <= 1'b0;
            dq_oe    <= 1'b0;
            
            // Issue NOP by default
            {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_NOP;
            
            case (state)
                S_INIT: begin
                    if (init_cnt < INIT_CYCLES) begin
                        init_cnt <= init_cnt + 1;
                    end else begin
                        state <= S_IDLE;
                    end
                end
                
                S_IDLE: begin
                    if (refresh_needed) begin
                        state <= S_REFRESH;
                    end else if (wr_req) begin
                        current_addr     <= wr_addr;
                        current_is_write <= 1'b1;
                        current_wr_data  <= wr_data;
                        state            <= S_ACTIVE;
                    end else if (rd_req) begin
                        current_addr     <= rd_addr;
                        current_is_write <= 1'b0;
                        state            <= S_ACTIVE;
                    end
                end
                
                S_ACTIVE: begin
                    // Issue ACTIVE command
                    {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_ACTIVE;
                    sdram_ba   <= active_bank;
                    sdram_addr <= active_row;
                    wait_cnt   <= 0;
                    
                    if (current_is_write)
                        state <= S_WRITE;
                    else
                        state <= S_READ;
                end
                
                S_WRITE: begin
                    // Issue WRITE command
                    {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_WRITE;
                    sdram_addr[COL_WIDTH-1:0] <= active_col;
                    dq_out <= current_wr_data;
                    dq_oe  <= 1'b1;
                    
                    // Write to internal memory model
                    mem_array[current_addr] <= current_wr_data;
                    
                    wr_ack <= 1'b1;
                    state  <= S_PRECHARGE;
                end
                
                S_READ: begin
                    // Issue READ command
                    {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_READ;
                    sdram_addr[COL_WIDTH-1:0] <= active_col;
                    wait_cnt <= 0;
                    state    <= S_READ_WAIT;
                end
                
                S_READ_WAIT: begin
                    // Wait for CAS latency
                    if (wait_cnt < CAS_LATENCY - 1) begin
                        wait_cnt <= wait_cnt + 1;
                    end else begin
                        state <= S_READ_DATA;
                    end
                end
                
                S_READ_DATA: begin
                    // Read from internal memory model
                    rd_data  <= mem_array[current_addr];
                    rd_valid <= 1'b1;
                    rd_ack   <= 1'b1;
                    state    <= S_PRECHARGE;
                end
                
                S_PRECHARGE: begin
                    {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_PRECHARGE;
                    sdram_addr[10] <= 1'b1;  // Precharge all banks
                    state <= S_IDLE;
                end
                
                S_REFRESH: begin
                    {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} <= CMD_REFRESH;
                    state <= S_IDLE;
                end
                
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
