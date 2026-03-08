module Temperature_System (
    input clk,          
    input reset,        
    output [15:0] temp_out, 
    output data_ready,  
    
    output i2c_scl,
    inout  i2c_sda
);

    // (Internal Wires)
    wire [7:0] w_i2c_data_to_master;
    wire [7:0] w_i2c_data_from_master;
    wire w_i2c_enable;
    wire w_i2c_rw;
    wire w_i2c_ready;

    ADT7420_Driver driver_inst (
        .clk(clk),
        .reset(reset),
        .temperature(temp_out),
        .data_valid(data_ready),
        .i2c_enable(w_i2c_enable),
        .i2c_rw(w_i2c_rw),
        .i2c_data_in(w_i2c_data_to_master),
        .i2c_data_out(w_i2c_data_from_master),
        .i2c_ready(w_i2c_ready)
    );

    i2c_master master_inst (
        .clk(clk),
        .reset(reset),
        .addr(7'h4B),          
        .data_in(w_i2c_data_to_master),
        .enable(w_i2c_enable),
        .rw(w_i2c_rw),
        .data_out(w_i2c_data_from_master),
        .ready(w_i2c_ready),
        .ack_error(),        
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda)
    );

endmodule