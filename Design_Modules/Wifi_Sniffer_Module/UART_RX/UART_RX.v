module UART_RX #(parameter DWIDTH = 8)(
 input 		  		 RX_IN,
 input 	[5:0] 		 prescale,
 input 		  		 PAR_EN,
 input 		  		 PAR_TYP,
 input 		  	     CLK,
 input 		  		 RST,
 output [DWIDTH-1:0] P_DATA,
 output  	  		 Parity_Error,
 output 	  		 Stop_Error,
 output  	  		 data_valid
);

// internal wires 
wire [3:0] bit_cnt;
wire [5:0] edge_cnt;
wire sampled_bit;
wire strt_glitch;
wire enable;
wire dat_samp_en;
wire deser_en;
wire strt_check_en;
wire par_check_en;
wire stp_check_en;

wire [DWIDTH-1:0] data;
wire [DWIDTH-1:0] deser_data;
wire       		  par_err_int;
wire       		  stp_err_int;
wire       	      data_valid_int;

// edge_bit_counter instantiation
edge_bit_counter u_counter (
 .enable(enable),
 .prescale(prescale),
 .PAR_EN(PAR_EN),  //add2
 .CLK(CLK),
 .RST(RST),
 .bit_cnt(bit_cnt),
 .edge_cnt(edge_cnt)
);

// data_sampling instantiation
data_sampling u_sampling (
 .RX_IN(RX_IN),
 .prescale(prescale),
 .edge_cnt(edge_cnt),
 .dat_samp_en(dat_samp_en),
 .CLK(CLK),
 .RST(RST),
 .sampled_bit(sampled_bit)
); 

//deserializer instantiation

deserializer u_deser (
 .sampled_bit(sampled_bit),
 .deser_en(deser_en),
 .data_valid(data_valid_int),
 .prescale(prescale),
 .bit_cnt(bit_cnt),
 .edge_cnt(edge_cnt),
 .CLK(CLK),
 .RST(RST),
 .P_DATA(deser_data),
 .data(data)
);

//start_check instantiation
start_check u_strt (
 .sampled_bit(sampled_bit),
 .strt_check_en(strt_check_en),
 .edge_cnt(edge_cnt),
 .prescale(prescale),
 .CLK(CLK),
 .RST(RST),
 .strt_glitch(strt_glitch)
);

// parity_check instantiation
parity_check u_par (
 .sampled_bit(sampled_bit),
 .data(data),
 .prescale(prescale),
 .par_check_en(par_check_en),
 .PAR_TYP(PAR_TYP),
 .bit_cnt(bit_cnt),
 .edge_cnt(edge_cnt),
 .CLK(CLK),
 .RST(RST),
 .par_err(par_err_int)
);

//stop_check instantiation
stop_check u_stp (
 .sampled_bit(sampled_bit),
 .stp_check_en(stp_check_en),
 .edge_cnt(edge_cnt),
 .prescale(prescale),
 .CLK(CLK),
 .RST(RST),
 .stp_err(stp_err_int)
);

// RX_FSM instantiation 
RX_FSM u_fsm (
 .RX_IN(RX_IN),
 .PAR_EN(PAR_EN),
 .bit_cnt(bit_cnt),
 .edge_cnt(edge_cnt),//add
 .prescale(prescale),//add
 .strt_glitch(strt_glitch),
 .par_err(par_err_int),
 .stp_err(stp_err_int),
 .CLK(CLK),
 .RST(RST),
 .enable(enable),
 .dat_samp_en(dat_samp_en),
 .deser_en(deser_en),
 .strt_check_en(strt_check_en),
 .par_check_en(par_check_en),
 .stp_check_en(stp_check_en),
 .data_valid(data_valid_int)
); 
// Connect internal wires to top-level outputs
assign P_DATA = deser_data;
assign Parity_Error = par_err_int;
assign Stop_Error = stp_err_int;
assign data_valid = data_valid_int;

endmodule
