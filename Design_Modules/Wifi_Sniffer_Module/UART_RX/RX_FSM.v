module RX_FSM (
 input 			  RX_IN,
 input 			  PAR_EN,
 input 		[3:0] bit_cnt,
 input		[5:0] edge_cnt,  //add
 input		[5:0] prescale,  //add
 input 			  strt_glitch,
 input 			  par_err,
 input			  stp_err,
 input 			  CLK,
 input 			  RST,
 output reg 	  enable,
 output reg 	  dat_samp_en,
 output reg 	  deser_en,
 output reg 	  strt_check_en,
 output reg 	  par_check_en,
 output reg 	  stp_check_en,
 output reg 	  data_valid
);

// state encoding
localparam IDLE = 3'b000;
localparam START_BIT = 3'b001;
localparam DATA = 3'b011;
localparam PARITY_BIT =3'b010;
localparam STOP_BIT = 3'b110;
localparam NEW_DATA = 3'b 111;

reg [2:0] current_state, next_state;
reg 	  data_valid_comb;

reg enable_c;
reg dat_samp_en_c;
reg deser_en_c;
reg strt_check_en_c;
reg par_check_en_c;
reg stp_check_en_c;


// state transition
always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		current_state <= IDLE;
	 end
	else
	 begin
		current_state <= next_state;
	 end
 end

// register output 
always @(posedge CLK or negedge RST)
 begin
	if(!RST)
	 begin
		data_valid <= 1'b0;
		enable <= 1'b0;
		dat_samp_en <= 1'b0;
		deser_en <= 1'b0;
		strt_check_en <= 1'b0;
		par_check_en <= 1'b0;
		stp_check_en <= 1'b0;
	 end
	else
	 begin
		enable <= enable_c;
		dat_samp_en <= dat_samp_en_c;
		deser_en <= deser_en_c;
		strt_check_en <= strt_check_en_c;
		par_check_en <= par_check_en_c;
		stp_check_en <= stp_check_en_c;
		/*if (data_valid_comb)
			data_valid <= data_valid_comb;
		else //if(current_state == START_BIT)
			data_valid <= 1'b0;*/
	 end
 end
 
// next state logic 
always @(*)
 begin
	case(current_state)
		IDLE: begin
			if(!RX_IN)
			 begin
				next_state = START_BIT;
			 end
			else
			 begin
				next_state = IDLE;
			 end
		end
		START_BIT: begin
			if(bit_cnt == 4'd0 & edge_cnt == (prescale - 1))
			 begin
				if(!strt_glitch)
				 begin
					next_state = DATA;
				 end
				else
				 begin
					next_state = IDLE;
				 end
			 end
			else
			 begin
				next_state = START_BIT;
			 end
		end
		DATA: begin
			if(bit_cnt == 4'd8 & edge_cnt == (prescale - 1)) 
			 begin
				if(PAR_EN)
				 begin
					next_state = PARITY_BIT;
				 end
				else
				 begin
					next_state = STOP_BIT;
				 end
			 end
			else
			 begin
				next_state = DATA;
			 end
		end
		PARITY_BIT: begin
			if(!par_err)
			 begin
				if(bit_cnt == 4'd9 & edge_cnt == (prescale - 1)) 
				 begin
					next_state = STOP_BIT;
				 end
				else 
				 begin
					next_state = PARITY_BIT;
				 end
			 end
			else
			 begin
				next_state = IDLE;
			 end
		end
		STOP_BIT: begin
			if(PAR_EN)
			 begin
				if (bit_cnt == 4'd10 && edge_cnt == (prescale-1))
					next_state = NEW_DATA;
				else
					next_state = STOP_BIT;
			 end
			else
			 begin
				if (bit_cnt == 4'd9 && edge_cnt == (prescale-1))
					next_state = NEW_DATA;
				else
					next_state = STOP_BIT;
			end
		end
		NEW_DATA: begin
			if(!RX_IN)
			 begin
				next_state = START_BIT;
			 end
			else
			 begin
				next_state = IDLE; // change IDLE to NEW_DATA
			 end
		end
		default: begin
			next_state = IDLE;
		end
	endcase
 end 
 
 // output logic 
always @(*)
 begin
	enable_c = 1'b0;
	dat_samp_en_c = 1'b0;
	deser_en_c = 1'b0;
	strt_check_en_c = 1'b0;
	par_check_en_c = 1'b0;
	stp_check_en_c = 1'b0;
	data_valid_comb = 1'b0;
	data_valid = 1'b0;
	case(current_state)
		IDLE: begin
		 if(!RX_IN) 
		  begin
			enable_c = 1'b1;
			dat_samp_en_c = 1'b1;
			strt_check_en_c = 1'b1;
		  end
		 else
		  begin
			enable_c = 1'b0;
			dat_samp_en_c = 1'b0;
			strt_check_en_c = 1'b0;
		  end
		 end

		START_BIT: begin
			enable_c = 1'b1;
			dat_samp_en_c = 1'b1;
			strt_check_en_c = 1'b1;
		end

		DATA: begin
			enable_c = 1'b1;
			dat_samp_en_c = 1'b1;
			deser_en_c = 1'b1;
		end

		PARITY_BIT: begin
			enable_c = 1'b1;
			dat_samp_en_c = 1'b1;
			if(edge_cnt == (prescale>>1))par_check_en_c = 1'b1; // last edit
		end

		STOP_BIT: begin
			enable_c      = 1'b1;
			dat_samp_en_c = 1'b1;
			if(edge_cnt == (prescale>>1))stp_check_en_c = 1'b1; // last edit
			if(!par_err && !stp_err && (edge_cnt == (prescale-1)))
			 begin	
				//data_valid_comb = 1'b1;
				data_valid = 1'b1;
			 end
			else
			 begin
				//data_valid_comb = 1'b0;
				data_valid = 1'b0;
			 end
		end
		
		NEW_DATA: begin
			if(!RX_IN)
			 begin
				enable_c = 1'b1;
				dat_samp_en_c = 1'b1;
				strt_check_en_c = 1'b1;
			 end
		end
		default: begin
			enable_c = 1'b0;
		end
	endcase
	
 end 

endmodule