module FIFO_RD (
  input             rinc,
  input             rclk,
  input             rrst_n,
  input      [3:0]  rq2_wptr,   // synced gray write pointer
  output wire [2:0] raddr,      // binary read address
  output reg  [3:0] rptr,       // gray coded read pointer
  output wire       rempty      // fifo empty flag
);

  reg [3:0] b_rptr;             // binary read pointer
  wire [3:0] g_rptr_comb;       // combinational gray pointer

  // binary -> gray conversion
  assign g_rptr_comb = b_rptr ^ (b_rptr >> 1);

  // increment binary read pointer
  always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n)
      b_rptr <= 4'b0;
    else if (!rempty && rinc)
      b_rptr <= b_rptr + 1'b1;
  end

  // read address (binary LSBs)
  assign raddr = b_rptr[2:0];

  // register gray read pointer
  always @(posedge rclk or negedge rrst_n) begin
    if (!rrst_n)
      rptr <= 4'b0;
    else
      rptr <= g_rptr_comb;
  end

  // empty flag generation 
  assign rempty = (rq2_wptr == g_rptr_comb);

endmodule
