`timescale 1ns/1ps

module scd30_tb;
    reg clk;
    reg reset;
    reg start;
    wire [143:0] sensor_data;
    wire done;
    wire scl;
    tri1 sda; 

    wire [31:0] co2_f, temp_f, hum_f;

    //--------------------------------
    // 1. Instance of Driver (DUT)
    //--------------------------------
    scd30_driver dut(
        .clk(clk), .reset(reset), .start_reading(start),
        .sensor_data(sensor_data), .done(done), .sda(sda), .scl(scl)
    );

    //--------------------------------
    // 2. Instance of Parser
    //--------------------------------
    scd30_parser parser_inst (
        .raw_data(sensor_data),
        .co2_float(co2_f),
        .temp_float(temp_f),
        .hum_float(hum_f)
    );

    //--------------------------------
    // Clock Generation
    //--------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    //--------------------------------
    // Fake sensor memory (Data to read)
    //--------------------------------
    reg [7:0] mock_mem [0:17];
    initial begin
        
        mock_mem[0] = 8'h43; mock_mem[1] = 8'hE1; mock_mem[2] = 8'hFF; // Byte0,1,CRC
        mock_mem[3] = 8'h40; mock_mem[4] = 8'h00; mock_mem[5] = 8'hFF; // Byte3,4,CRC
        
        // Temp = 25.5 C -> Hex: 41CC0000
        mock_mem[6] = 8'h41; mock_mem[7] = 8'hCC; mock_mem[8] = 8'hFF;
        mock_mem[9] = 8'h00; mock_mem[10]= 8'h00; mock_mem[11]= 8'hFF;
        
        // Hum = 50.25 % -> Hex: 42490000
        mock_mem[12]= 8'h42; mock_mem[13]= 8'h49; mock_mem[14]= 8'hFF;
        mock_mem[15]= 8'h00; mock_mem[16]= 8'h00; mock_mem[17]= 8'hFF;
    end

    //--------------------------------
    // Master Signals Monitor
    //--------------------------------
    wire [2:0] mstate = dut.master_inst.state;
    wire [3:0] mbit   = dut.master_inst.bit_cnt;
    reg sda_drive, sda_out;
    assign sda = (sda_drive) ? sda_out : 1'bz;

    integer b_idx;
    reg prev_byte_done;

    initial begin
        sda_drive = 0; sda_out = 1;
        b_idx = 0; prev_byte_done = 0;
    end

    //--------------------------------
    // Slave behavior (Emulating SCD30)
    //--------------------------------
    always @(*) begin
        sda_drive = 0; sda_out = 1;
        if (mstate == 3) begin // ACK Check
            if (scl) begin sda_drive = 1; sda_out = 0; end
        end
        else if (mstate == 5 && b_idx < 18) begin // READ Mode
            sda_drive = 1;
            sda_out = mock_mem[b_idx][mbit];
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            b_idx <= 0; prev_byte_done <= 0;
        end else begin
            prev_byte_done <= dut.master_inst.byte_done;
            if (dut.master_inst.byte_done && !prev_byte_done) begin
                if (dut.master_inst.rw == 1) b_idx <= b_idx + 1;
            end
            if (mstate == 0) b_idx <= 0;
        end
    end

    //--------------------------------
    // Main Stimulus & Display Result
    //--------------------------------
    initial begin
        reset = 1; start = 0;
        #100 reset = 0;
        #200 start = 1;
        #20  start = 0;

        wait(done);
        #100;

        $display("------------------------------------------------");
        $display("RAW DATA (HEX): %h", sensor_data);
        $display("------------------------------------------------");
        $display("======= SCD30 REAL-TIME READINGS =======");
        $display("CO2 Concentration : %f ppm", $bitstoshortreal(co2_f));
        $display("Temperature       : %f C",   $bitstoshortreal(temp_f));
        $display("Humidity          : %f %%",  $bitstoshortreal(hum_f));
        $display("========================================");
        
        #100;
        $finish;
    end

    initial begin
        #10000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule