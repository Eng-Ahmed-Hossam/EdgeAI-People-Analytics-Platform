module ASYNC_FIFO #(parameter DATA_WIDTH = 8)(
 input 					 W_CLK,
 input 					 W_RST,
 input 					 W_INC,
 input 					 R_CLK,
 input 				 	 R_RST,
 input 					 R_INC,
 input 	[DATA_WIDTH-1:0] WR_DATA,
 output  				 FULL,
 output  				 EMPTY,
 output [DATA_WIDTH-1:0] RD_DATA
);

// internal signals
wire [2:0] waddr;
wire [2:0] raddr;
wire [3:0] wptr;
wire [3:0] rptr;
wire [3:0] rq2_wptr;
wire [3:0] wq2_rptr;

wire [DATA_WIDTH-1:0] RD_DATA_int;
wire FULL_int;
wire EMPTY_int;

assign FULL = FULL_int;
assign EMPTY = EMPTY_int;
assign RD_DATA = RD_DATA_int;


// write pointer
FIFO_WR u_wr_ptr (
 .winc(W_INC),
 .wclk(W_CLK),
 .wrst_n(W_RST),
 .wq2_rptr(wq2_rptr),
 .waddr(waddr),
 .wptr(wptr),
 .wfull(FULL_int)
);

// read pointer
FIFO_RD u_rd_ptr (
 .rinc(R_INC),
 .rclk(R_CLK),
 .rrst_n(R_RST),
 .rq2_wptr(rq2_wptr),
 .raddr(raddr),
 .rptr(rptr),
 .rempty(EMPTY_int)
);

// write synchronizer
DF_SYNC u0_sync (
 .ptr(wptr),
 .clk(R_CLK),
 .rst_n(R_RST),
 .sync(rq2_wptr)
);

// resd synchronizer
DF_SYNC u1_sync (
 .ptr(rptr),
 .clk(W_CLK),
 .rst_n(W_RST),
 .sync(wq2_rptr)
);

// memory
FIFO_MEM u_mem (
 .wdata(WR_DATA),
 .waddr(waddr),
 .raddr(raddr),
 .winc(W_INC),
 .wfull(FULL_int),
 .wclk(W_CLK),
 .wrst_n(W_RST),
 .rdata(RD_DATA_int)
);

endmodule
