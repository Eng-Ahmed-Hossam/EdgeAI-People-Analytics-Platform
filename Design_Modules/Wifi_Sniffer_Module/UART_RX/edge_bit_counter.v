module edge_bit_counter(
 input            enable,
 input      [5:0] prescale,
 input			  PAR_EN,
 input            CLK,
 input            RST,
 output reg [3:0] bit_cnt,  // frame 4 bits  0:10  (11)
 output reg [5:0] edge_cnt // prescale 4 (0:3) 2 bits, 8(0:7) 3 bits, 16(0:15) 4 bits, 32(0:31) 5 bits
);

// register output
always @(posedge CLK or negedge RST)
 begin
  if(!RST)
   begin
    bit_cnt <= 5'b0;
	edge_cnt <= 5'b0;
   end
  else if(enable) 
   begin
    if(edge_cnt == prescale - 1)
	 begin
      edge_cnt <= 0;
	  if(PAR_EN)
	   begin
		if(bit_cnt == 10)       
			bit_cnt <= 0;
		else
			bit_cnt <= bit_cnt + 1;
	   end
	  else
       begin
		if(bit_cnt == 9)       
			bit_cnt <= 0;
		else
			bit_cnt <= bit_cnt + 1;
	   end
     end
    else if (edge_cnt < (prescale - 1))
	 begin
		edge_cnt <= edge_cnt + 1;
     end
	else
	 begin
		edge_cnt <= 5'b0;
		bit_cnt <= 5'b0;
	 end
   end
  /*else
   begin
	edge_cnt <= 5'b0; // last edit 
	bit_cnt <= 5'b0;
   end*/
 end

endmodule