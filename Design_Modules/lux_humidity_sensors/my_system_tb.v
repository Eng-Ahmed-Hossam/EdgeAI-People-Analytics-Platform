`timescale 1ns/1ps
module my_system_tb;
    reg clk, rst;
    wire sda, scl;
    wire [15:0] lux, hum;

    assign (weak1, weak0) sda = 1'b1;
    assign (weak1, weak0) scl = 1'b1;

    digital_sensor_system #(.SIM_SPEEDUP(1)) dut (
        .clk(clk), .rst(rst), .sda(sda), .scl(scl), 
        .final_lux(lux), .final_hum(hum)
    );

    always #5 clk = (clk === 1'b0);

    initial begin
        clk = 0; rst = 1;
        #100 rst = 0;
        $display("--- Simulation Started ---");

        // Step 1: Wait for Lux Read
        wait(dut.state == 1);
        $display("[%t] Reading Lux...", $time);
        #1000;
        force dut.i2c_inst.read_data = 16'h04B0; // 1000 Lux
        force dut.i2c_inst.done = 1;
        #20 release dut.i2c_inst.done; release dut.i2c_inst.read_data;

        // Step 2: Wait for Humidity Read
       // Step 2: Wait for Humidity Read
        wait(dut.state == 2);
        force dut.i2c_inst.read_data = 16'h8000;
        force dut.i2c_inst.done = 1;
        #2000; // WAIT A LONG TIME
        $display("CHECK: data_buffer is %h", dut.data_buffer); // Add this line!
        release dut.i2c_inst.done;
        release dut.i2c_inst.read_data;

        #5000;
        $display("--------------------------------------");
        $display("FINAL SUCCESS -> Lux: %d | Humidity: %d %%", lux, hum);
        $display("--------------------------------------");
        $finish;
    end
endmodule