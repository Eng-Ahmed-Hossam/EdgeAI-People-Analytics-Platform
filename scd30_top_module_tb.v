`timescale 1ns/1ps
module scd30_top_tb();
    reg clk, reset_n, read_btn;
    wire [15:0] co2_leds;
    wire done_tick, i2c_scl, i2c_sda;

    scd30_top uut (
        .clk(clk), .reset_n(reset_n), .read_btn(read_btn),
        .co2_leds(co2_leds), .done_tick(done_tick),
        .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    always #5 clk = ~clk;

    reg [7:0] sensor_data [0:1];
    integer b_idx = 0;
    integer bit_idx = 7;

    initial begin
        sensor_data[0] = 8'h04; 
        sensor_data[1] = 8'hB1;
    end

    // Sensor behavior logic
    assign i2c_sda = (uut.master_inst.state == 3 || uut.master_inst.state == 6) ? 1'b0 : 
                     (uut.master_inst.state == 5) ? sensor_data[b_idx][bit_idx] : 1'bz;

    always @(negedge i2c_scl) begin
        if (uut.master_inst.state == 5) begin
            #10; 
            if (bit_idx > 0) bit_idx <= bit_idx - 1;
            else begin
                bit_idx <= 7;
                b_idx <= (b_idx + 1) % 2;
            end
        end
    end

    initial begin
        clk = 0; reset_n = 0; read_btn = 0;
        #100 reset_n = 1;
        #200 read_btn = 1; #100 read_btn = 0;
        
        wait(done_tick == 1);
        #100;
        $display("---------------------------------------");
        $display("FINAL RESULT: 0x%h (%d ppm)", co2_leds, co2_leds);
        if (co2_leds == 16'h04B1) 
            $display("TEST RESULT: SUCCESS!");
        else 
            $display("TEST RESULT: FAILED! (Got 0x%h)", co2_leds);
        $display("---------------------------------------");
        #1000 $stop;
    end
endmodule