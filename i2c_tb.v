`timescale 1ns/1ps

module i2c_master_tb();
    reg clk;
    reg reset;
    reg [6:0] addr;
    reg [7:0] data_in;
    reg enable;
    reg rw;
    
    wire [7:0] data_out;
    wire ready;
    wire ack_error;
    wire i2c_scl;
    wire i2c_sda;

    i2c_master uut (
        .clk(clk), .reset(reset), .addr(addr),
        .data_in(data_in), .enable(enable), .rw(rw),
        .data_out(data_out), .ready(ready), 
        .ack_error(ack_error), .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always #5 clk = ~clk;

    // (Slave Mock)
    reg [7:0] sensor_mock_data = 8'h2A; 
    integer bit_idx = 7;

    assign i2c_sda = (uut.state == 3 || (uut.state == 6 && uut.rw == 0)) ? 1'b0 : 
                     (uut.state == 5) ? sensor_mock_data[bit_idx] : 1'bz;

    always @(negedge i2c_scl) begin
        if (uut.state == 5) begin
            if (bit_idx > 0) bit_idx <= bit_idx - 1;
            else bit_idx <= 7;
        end else bit_idx <= 7;
    end

    initial begin
        // 1. Initial State
        clk = 0; reset = 1; enable = 0; addr = 0; data_in = 0; rw = 0;

        $display("--------------------------------------------------");
        $display("STARTING REFINED I2C SIMULATION");
        $display("--------------------------------------------------");

        #100 reset = 0;
        #100 wait(ready == 1); 

        // 2. Write Operation
        #100;
        addr = 7'h4B; data_in = 8'h01; rw = 0; enable = 1;
        #100;
        if (ready) wait(ready == 0); 
        enable = 0; 
        
        wait(ready == 1);  
        $display("Time: %0t | Action: Write Operation Finished", $time);

        #5000; 

        // 3. Read Operation
        #100;
        addr = 7'h4B; rw = 1; enable = 1;
        #100;
        if (ready) wait(ready == 0);
        enable = 0;

        wait(ready == 1); 
        $display("Time: %0t | SUCCESS: Data Received from Sensor = 0x%h", $time, data_out);

        $display("--------------------------------------------------");
        $display("SIMULATION FINISHED SUCCESSFULLY");
        $display("--------------------------------------------------");
        #1000 $stop;
    end

    initial begin
        $monitor("Time: %0t | State: %0d | SCL: %b | SDA: %b | Ready: %b", $time, uut.state, i2c_scl, i2c_sda, ready);
    end

endmodule