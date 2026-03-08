`timescale 1ns/1ps

module top_system_tb();
    reg clk;
    reg reset;
    wire [15:0] temp_out;
    wire data_ready;
    wire i2c_scl;
    wire i2c_sda;

    Temperature_System uut (
        .clk(clk),
        .reset(reset),
        .temp_out(temp_out),
        .data_ready(data_ready),
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda)
    );

    always #5 clk = ~clk;

   
    reg [15:0] fake_sensor_data = 16'h0B45; 
    integer bit_idx = 15;

   assign i2c_sda = (uut.master_inst.state == 3 || uut.master_inst.state == 6) ? 1'b0 : 
                     (uut.master_inst.state == 5) ? fake_sensor_data[bit_idx] : 1'bz;

    always @(negedge i2c_scl) begin
        if (uut.master_inst.state == 5) begin
            if (bit_idx > 0) bit_idx <= bit_idx - 1;
            else bit_idx <= 15; 
        end
    end

    initial begin
        clk = 0;
        reset = 1;
        $display("--------------------------------------------------");
        $display("STARTING FULL SYSTEM SIMULATION (Master + Driver)");
        $display("--------------------------------------------------");

        #100 reset = 0;

        wait(data_ready == 1);
        
        $display("Time: %t | SUCCESS!", $time);
        $display("Temperature Received: %h", temp_out);
        $display("Expected Value: 0B45");
        
        #2000;
        $display("--------------------------------------------------");
        $display("FULL SYSTEM TEST COMPLETED");
        $display("--------------------------------------------------");
        $stop;
    end

    initial begin
        $monitor("Time: %t | Temp_Raw: %h | Ready_Flag: %b", $time, temp_out, data_ready);
    end

endmodule