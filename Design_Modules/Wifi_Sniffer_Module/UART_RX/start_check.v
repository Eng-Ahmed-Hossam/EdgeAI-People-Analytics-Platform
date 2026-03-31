module start_check (
 input 			  sampled_bit,
 input 			  strt_check_en,
 input 		[5:0] edge_cnt,
 input		[5:0] prescale,
 input 			  CLK,
 input      	  RST,
 output reg 	  strt_glitch
);

always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		strt_glitch <= 1'b0;
	 end
	else if (strt_check_en)
	 begin
		if(edge_cnt > ((prescale >> 1)+6'd1)) 
		 begin
			strt_glitch <= (!sampled_bit) ? 1'b0 : 1'b1;
		 end
	 end
	 
 end
 
endmodule