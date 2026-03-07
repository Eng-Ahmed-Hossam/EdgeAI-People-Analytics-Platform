// FINAL PRODUCTION CODE (FOR FPGA FLASHING)
module sensor_system_final (
    input  wire        clk,   
    input  wire        rst,
    inout  wire        sda,
    inout  wire        scl,
    output wire [15:0] lux_out,
    output wire [15:0] hum_out     
); 
    // SIM_SPEEDUP = 0 for real FPGA hardware   ----> 20ms
    digital_sensor_system #(.SIM_SPEEDUP(0)) core_inst (
        .clk(clk),
        .rst(rst),
        .sda(sda),
        .scl(scl),
        .final_lux(lux_out),
        .final_hum(hum_out)       
    );
endmodule