module FIFO_MEM #(
  parameter DATA_WIDTH = 8
)(
  input      [DATA_WIDTH-1:0] wdata,
  input      [2:0]            waddr,   
  input      [2:0]            raddr,
  input                       winc,
  input                       wfull,
  input                       wclk,
  input                       wrst_n,
  output     [DATA_WIDTH-1:0] rdata
);

  integer i;
  reg [DATA_WIDTH-1:0] MEM [0:7];  

  // synchronous write + reset
  always @(posedge wclk or negedge wrst_n) begin
    if(!wrst_n) begin
      for(i=0; i<8; i=i+1)
        MEM[i] <= {DATA_WIDTH{1'b0}};
    end
    else if(winc & !wfull) begin
      MEM[waddr] <= wdata;
    end
  end

  // combinational read
  assign rdata = MEM[raddr];

endmodule
