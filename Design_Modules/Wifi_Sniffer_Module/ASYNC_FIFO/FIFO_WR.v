module FIFO_WR (
  input              winc,
  input              wclk,
  input              wrst_n,
  input       [3:0]  wq2_rptr,   // synced gray read pointer
  output wire [2:0]  waddr,      // binary write address
  output reg  [3:0]  wptr,       // gray coded write pointer
  output wire        wfull       // fifo full flag
);

  reg [3:0] b_wptr;              // binary write pointer
  wire [3:0] g_wptr_comb;        // combinational gray pointer

  // binary -> gray conversion
  assign g_wptr_comb = b_wptr ^ (b_wptr >> 1);

  // increment binary write pointer
  always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n)
      b_wptr <= 4'b0;
    else if (!wfull && winc)
      b_wptr <= b_wptr + 1'b1;
  end

  // write address (binary LSBs)
  assign waddr = b_wptr[2:0];

  // register gray write pointer
  always @(posedge wclk or negedge wrst_n) begin
    if (!wrst_n)
      wptr <= 4'b0;
    else
      wptr <= g_wptr_comb;
  end

  // full flag generation 
  assign wfull = (wq2_rptr[3]   != g_wptr_comb[3]) &&
                 (wq2_rptr[2]   != g_wptr_comb[2]) &&
                 (wq2_rptr[1:0] == g_wptr_comb[1:0]);

endmodule
