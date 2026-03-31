module data_sampling (
 input            RX_IN,
 input      [5:0] prescale,
 input      [5:0] edge_cnt,
 input            dat_samp_en,
 input            CLK,
 input            RST,
 output reg       sampled_bit
);

reg sampling1, sampling2, sampling3;

always @(posedge CLK or negedge RST) begin
    if(!RST) 
	 begin
        sampling1 <= 1'b0;
        sampling2 <= 1'b0;
        sampling3 <= 1'b0;
        sampled_bit <= 1'b0;
     end 
	else if (dat_samp_en) 
	 begin
		if(edge_cnt == ((prescale >> 1)-6'd2)) 
		 begin
			sampling1 <= RX_IN;
		 end
        else if(edge_cnt == ((prescale >> 1)-6'd1)) 
		 begin
			sampling2 <= RX_IN;
		 end
        else if(edge_cnt == (prescale >> 1)) 
		 begin
			sampling3 <= RX_IN;
		 end
        
        sampled_bit <= (sampling1 & sampling2 & sampling3) |
                       (!sampling1 & sampling2 & sampling3) |
                       (sampling1 & !sampling2 & sampling3) |
                       (sampling1 & sampling2 & !sampling3);
     end 
	else 
	 begin
        sampled_bit <= 1'b0;
    end
end

endmodule 