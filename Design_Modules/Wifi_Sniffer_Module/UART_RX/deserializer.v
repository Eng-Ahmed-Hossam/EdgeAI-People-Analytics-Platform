module deserializer #(parameter DWIDTH = 8)(
 input            		 sampled_bit,
 input            		 deser_en,
 input			  		 data_valid,
 input      [5:0] 	 	 prescale,
 input      [3:0] 		 bit_cnt,
 input      [5:0] 		 edge_cnt,
 input            	 	 CLK,
 input            		 RST,
 output reg [DWIDTH-1:0] P_DATA,
 output reg [DWIDTH-1:0] data
);

always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
        data <= 'd0;
		P_DATA <= 'd0; // last edit change sequential with combinational
	 end
    else if(deser_en && (edge_cnt  == (prescale - 6'b1)))
	 begin
        data <= {sampled_bit,data[DWIDTH-1:1]};
	 end
 end
 
/*always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		P_DATA <= 'd0;
	 end
    else if(data_valid)
	 begin
        P_DATA <= data;
	 end
	/*else
	 begin
		P_DATA <= 'd0;
	 end*/
 //end
always @(*)
 begin
	if(data_valid)
	 begin
        P_DATA <= data;
	 end
	else
	 begin
		P_DATA <= 'd0;
	 end
 end
 
endmodule 