`timescale 1ns/1ps

module i2c_master_tb();
    reg clk, reset, enable, rw;
    reg [6:0] addr;
    reg [7:0] data_in;
    wire [7:0] data_out;
    wire ready, i2c_scl, i2c_sda;

    i2c_master uut (
        .clk(clk), .reset(reset), .addr(addr), .data_in(data_in),
        .enable(enable), .rw(rw), .data_out(data_out), .ready(ready),
        .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always #5 clk = ~clk; 

    
    reg [7:0] co2_data [0:2]; 
    integer b_ptr = 0;
    integer bit_ptr = 7;

    initial begin
        co2_data[0] = 8'h01; // CO2 MSB
        co2_data[1] = 8'hF4; // CO2 LSB
        co2_data[2] = 8'h33; // CRC Checksum
    end

    assign i2c_sda = (uut.state == 3 || uut.state == 6) ? 1'b0 : // Send ACK
                     (uut.state == 5) ? co2_data[b_ptr][bit_ptr] : 1'bz;

    always @(negedge i2c_scl) begin
        if (uut.state == 5) begin
            #1; 
            if (bit_ptr > 0) begin
                bit_ptr <= bit_ptr - 1;
            end else begin
                bit_ptr <= 7;
                b_ptr <= (b_ptr + 1) % 3; 
            end
        end else if (uut.state == 0) begin
            bit_ptr <= 7;
            b_ptr <= 0;
        end
    end

    initial begin
        clk = 0; reset = 1; enable = 0; addr = 7'h61; rw = 0;
        #100 reset = 0;
        wait(ready == 1);

        $display("--- Starting SCD30 CO2 Reading Simulation ---");

        #100 rw = 1; enable = 1;
        #100 wait(ready == 0); enable = 0;
        wait(ready == 1);
        $display("Time: %0t | CO2 MSB Received: 0x%h", $time, data_out);

        #5000; 

        #100 rw = 1; enable = 1;
        #100 wait(ready == 0); enable = 0;
        wait(ready == 1);
        $display("Time: %0t | CO2 LSB Received: 0x%h", $time, data_out);

        $display("--- Reading Finished ---");
        $display("Final CO2 Value (Hex): 0x01F4 | Decimal: 500 ppm");
        
        #1000 $stop;
    end
endmodule   