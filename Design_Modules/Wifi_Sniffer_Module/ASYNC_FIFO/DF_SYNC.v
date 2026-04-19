module DF_SYNC (
 input  [3:0] ptr,
 input		  clk,
 input		  rst_n,
 output [3:0] sync
);

reg [3:0] sync_0;
reg [3:0] sync_1;

always @(posedge clk or negedge rst_n)
 begin
	if(!rst_n)
	 begin
		sync_0 <= 'b0;
		sync_1 <= 'b0;
	 end
	else
	 begin
		sync_0 <= ptr;
		sync_1 <= sync_0;
	 end
 end
 
assign sync = sync_1;

endmodule