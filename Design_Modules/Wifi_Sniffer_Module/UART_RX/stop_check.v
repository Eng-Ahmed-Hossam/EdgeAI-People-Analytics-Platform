module stop_check (
 input 		sampled_bit,
 input 		stp_check_en,
 input 		[5:0] edge_cnt,
 input		[5:0] prescale,
 input 		CLK,
 input      RST,
 output reg stp_err
);

always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		stp_err <= 1'b0;
	 end
	else if (stp_check_en)
	 begin
		if(edge_cnt > ((prescale >> 1)+6'd1)) 
		 begin
			stp_err <= (sampled_bit) ? 1'b0 : 1'b1;
		 end
	 end
 end
 
endmodule