module parity_check #(parameter DWIDTH = 8)(
 input 			  		 sampled_bit,
 input 			  		 par_check_en,
 input            		 PAR_TYP,
 input	    [5:0] 		 prescale,
 input      [DWIDTH-1:0] data,
 input      [3:0] 		 bit_cnt,
 input		[5:0] 		 edge_cnt,
 input 			  		 CLK,
 input      	  		 RST,
 output reg 	  		 par_err
);


always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		par_err <= 1'b0;
	 end
	else if(par_check_en) 
	 begin
        if(bit_cnt == 4'd9 & edge_cnt == ((prescale >> 1)+6'd1)) 
		 begin
			if(!PAR_TYP) // even parity
				par_err <= (sampled_bit == ^ data) ? 1'b0 : 1'b1;
			else         // odd parity
				par_err <= (sampled_bit == ~ (^ data)) ? 1'b0 : 1'b1;   
         end
     end
 end

endmodule