`timescale 1ns/1ps

module adt7420_driver_tb();
    reg clk, reset;
    wire [15:0] temperature;
    wire data_valid;
    
    wire i2c_enable, i2c_rw;
    wire [7:0] i2c_data_in;
    reg [7:0] i2c_data_out;
    reg i2c_ready;

    ADT7420_Driver uut (
        .clk(clk), .reset(reset),
        .temperature(temperature), .data_valid(data_valid),
        .i2c_enable(i2c_enable), .i2c_rw(i2c_rw),
        .i2c_data_in(i2c_data_in), .i2c_data_out(i2c_data_out),
        .i2c_ready(i2c_ready)
    );

    always #5 clk = ~clk;

    initial begin
        
        clk = 0; reset = 1; i2c_ready = 1; i2c_data_out = 8'h00;
        #100 reset = 0;

        $display("--- Starting Driver Test ---");

        wait(i2c_enable == 1);
        $display("Time: %t | Driver requested Write to Reg: %h", $time, i2c_data_in);
        #100 i2c_ready = 0; 
        #500 i2c_ready = 1; 

        wait(i2c_enable == 1 && i2c_rw == 1);
        $display("Time: %t | Driver requested Read MSB", $time);
        i2c_data_out = 8'h1E; 
        #100 i2c_ready = 0;
        #500 i2c_ready = 1;

        wait(i2c_enable == 1 && i2c_rw == 1);
        $display("Time: %t | Driver requested Read LSB", $time);
        i2c_data_out = 8'h05; 
        #100 i2c_ready = 0;
        #500 i2c_ready = 1;

        
        wait(data_valid == 1);
        $display("-------------------------------------------");
        $display("SUCCESS: Full Temperature Received = %h", temperature);
        $display("Expected: 1E05");
        $display("-------------------------------------------");

        #100 $stop;
    end
endmodule