`timescale 1ns/1ps
module scd30_driver (
    input  wire        clk,
    input  wire        reset,
    input  wire        start_reading,
    output reg  [143:0] sensor_data,
    output reg         done,
    inout  wire        sda,
    output wire        scl
);
    localparam IDLE = 2'd0, SEND_CMD = 2'd1, READ_DATA = 2'd2;
    reg [1:0] state;
    reg [1:0] step;

    reg        i2c_en;       
    reg        i2c_rw;
    reg [7:0]  i2c_data_in;
    reg [4:0]  b_read;
    reg [4:0]  b_write;

    wire [7:0] i2c_data_out;
    wire       i2c_ready;
    wire       i2c_byte_done;

    i2c_master master_inst (
        .clk(clk), .reset(reset), .addr(7'h61), .data_in(i2c_data_in),
        .enable(i2c_en), .rw(i2c_rw), .bytes_to_read(b_read), .bytes_to_write(b_write),
        .data_out(i2c_data_out), .ready(i2c_ready), .ack_error(), .i2c_scl(scl), .i2c_sda(sda),
        .byte_done(i2c_byte_done)
    );

    reg en_req;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            i2c_en <= 0; en_req <= 0;
        end else begin
            if (en_req) begin i2c_en <= 1; en_req <= 0; end 
            else i2c_en <= 0;
        end
    end

    reg [4:0] recv_cnt; 
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE; step <= 0; i2c_rw <= 0; b_read <= 0; b_write <= 0;
            i2c_data_in <= 8'h00; done <= 0; sensor_data <= 144'h0; recv_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    done <= 0;
                    if (start_reading) begin
                        state <= SEND_CMD; step <= 0; recv_cnt <= 0; sensor_data <= 144'h0;
                    end
                end
                SEND_CMD: begin
                    if (step == 0 && master_inst.ready) begin
                        b_write <= 5'd2; i2c_rw <= 1'b0; i2c_data_in <= 8'h03;
                        en_req <= 1; step <= 1;
                    end else if (step == 1 && i2c_byte_done) begin
                        i2c_data_in <= 8'h00; step <= 2;
                    end else if (step == 2 && i2c_byte_done) begin
                        state <= READ_DATA; step <= 0; recv_cnt <= 0;
                    end
                end
                READ_DATA: begin
                    if (step == 0 && master_inst.ready) begin
                        b_read <= 5'd18; i2c_rw <= 1'b1; en_req <= 1; step <= 1; recv_cnt <= 0;
                    end else if (i2c_byte_done && step == 1) begin
                        sensor_data <= { sensor_data[135:0], i2c_data_out };
                        recv_cnt <= recv_cnt + 1;
                        if (recv_cnt + 1 >= 18) step <= 2; 
                    end else if (step == 2 && master_inst.ready) begin
                        done <= 1; state <= IDLE;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule