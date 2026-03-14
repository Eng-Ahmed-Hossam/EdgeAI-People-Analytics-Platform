`timescale 1ns/1ps

module i2c_master_tb;

    reg        clk;
    reg        reset;
    reg [6:0]  addr;
    reg [7:0]  data_in;
    reg        enable;
    reg        rw;
    reg [4:0]  bytes_to_read;
    reg [4:0]  bytes_to_write;
    
    wire [7:0] data_out;
    wire       ready;
    wire       ack_error;
    wire       i2c_scl;
    wire       byte_done;
    tri1       i2c_sda; 

    
    //---------------------------------------------------------
    i2c_master uut (
        .clk(clk),
        .reset(reset),
        .addr(addr),
        .data_in(data_in),
        .enable(enable),
        .rw(rw),
        .bytes_to_read(bytes_to_read),
        .bytes_to_write(bytes_to_write),
        .data_out(data_out),
        .ready(ready),
        .ack_error(ack_error),
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda),
        .byte_done(byte_done)
    );

    
    initial clk = 0;
    always #5 clk = ~clk; 

   
    reg sda_drive;
    assign i2c_sda = (sda_drive) ? 1'b0 : 1'bz;

    always @(*) begin
        if (uut.state == 3'd3) begin
            if (i2c_scl) 
                sda_drive = 1; 
            else 
                sda_drive = 0;
        end else begin
            sda_drive = 0;
        end
    end

    
    initial begin
        reset = 1;
        enable = 0;
        addr = 7'h61;      //  SCD30 address
        data_in = 8'h03;   
        rw = 0;           
        bytes_to_write = 1;
        bytes_to_read = 0;

        #100;
        reset = 0;
        #100;

        @(posedge clk);
        enable = 1;
        #20;
        enable = 0;

        wait(ready == 1);
        #500;

        $display("Test Completed. check waves for SCL and SDA");
        $finish;
    end

    initial begin
        $monitor("Time=%0t | State=%0d | SCL=%b | SDA=%b | Ready=%b", 
                 $time, uut.state, i2c_scl, i2c_sda, ready);
    end

endmodule