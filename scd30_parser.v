`timescale 1ns/1ps

module scd30_parser (
    input  wire [143:0] raw_data,    // 18 bit from driver
    output wire [31:0]  co2_float,   // co2 value decimal
    output wire [31:0]  temp_float,  // temp value decimal
    output wire [31:0]  hum_float    // humdity val in decimal
);

        assign co2_float = {raw_data[143:136], raw_data[135:128], raw_data[119:112], raw_data[111:104]};

        assign temp_float = {raw_data[95:88], raw_data[87:80], raw_data[71:64], raw_data[63:56]};

        assign hum_float = {raw_data[47:40], raw_data[39:32], raw_data[23:16], raw_data[15:8]};

endmodule