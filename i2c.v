`timescale 1ns/1ps
module i2c_master (
    input  wire        clk,
    input  wire        reset,
    input  wire [6:0]  addr,
    input  wire [7:0]  data_in,
    input  wire        enable,   
    input  wire        rw,       
    input  wire [4:0]  bytes_to_read,
    input  wire [4:0]  bytes_to_write,
    output reg  [7:0]  data_out,
    output reg         ready,
    output reg         ack_error,
    output reg         i2c_scl,
    inout  wire        i2c_sda,
    output reg         byte_done
);

    localparam IDLE       = 3'd0;
    localparam START      = 3'd1;
    localparam ADDR       = 3'd2;
    localparam ACK_CHECK  = 3'd3;
    localparam WRITE      = 3'd4;
    localparam READ       = 3'd5;
    localparam MASTER_ACK = 3'd6;
    localparam STOP       = 3'd7;

    reg [2:0]  state;
    reg [2:0]  next_state_after_ack;
    reg [3:0]  bit_cnt;
    reg [15:0] clk_cnt;
    reg [4:0]  b_cnt;       
    reg        sda_out;
    reg        sda_en;

    assign i2c_sda = (sda_en) ? sda_out : 1'bz;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state         <= IDLE;
            i2c_scl       <= 1;
            ready         <= 1;
            sda_en        <= 0;
            sda_out       <= 1;
            clk_cnt       <= 0;
            ack_error     <= 0;
            byte_done     <= 0;
            b_cnt         <= 0;
            data_out      <= 8'h00;
            bit_cnt       <= 7;
            next_state_after_ack <= IDLE;
        end else begin
            byte_done <= 0;
            clk_cnt <= clk_cnt + 1;

            case (state)
                IDLE: begin
                    ready <= 1; i2c_scl <= 1; sda_en <= 0; clk_cnt <= 0;
                    b_cnt <= 0; ack_error <= 0; bit_cnt <= 7;
                    if (enable) begin
                        ready <= 0; clk_cnt <= 0; state <= START;
                    end
                end
                START: begin
                    sda_en <= 1; sda_out <= 0;
                    if (clk_cnt >= 10) begin
                        clk_cnt <= 0; i2c_scl <= 0; bit_cnt <= 7; state <= ADDR;
                    end
                end
                ADDR: begin
                    sda_en <= 1;
                    sda_out <= (bit_cnt >= 1) ? addr[bit_cnt-1] : rw;
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 12) i2c_scl <= 0;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else begin
                            sda_en <= 0;
                            next_state_after_ack <= rw ? READ : WRITE;
                            state <= ACK_CHECK;
                        end
                    end
                end
                ACK_CHECK: begin
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 9) begin
                        if (i2c_sda !== 1'b0) ack_error <= 1;
                    end
                    if (clk_cnt == 12) i2c_scl <= 0;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0;
                        if (ack_error) state <= STOP;
                        else begin
                            if (next_state_after_ack == WRITE) begin
                                bit_cnt <= 7; state <= WRITE;
                            end else if (next_state_after_ack == READ) begin
                                bit_cnt <= 7; data_out <= 8'h00; b_cnt <= 0; state <= READ;
                            end else state <= STOP;
                        end
                    end
                end
                WRITE: begin
                    sda_en <= 1; sda_out <= data_in[bit_cnt];
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 12) i2c_scl <= 0;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else begin
                            b_cnt <= b_cnt + 1; byte_done <= 1; sda_en <= 0;
                            if (b_cnt + 1 < bytes_to_write) next_state_after_ack <= WRITE;
                            else next_state_after_ack <= (rw ? READ : STOP);
                            state <= ACK_CHECK;
                        end
                    end
                end
                READ: begin
                    sda_en <= 0;
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 9) data_out[bit_cnt] <= i2c_sda;
                    if (clk_cnt == 12) i2c_scl <= 0;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else begin
                            b_cnt <= b_cnt + 1; byte_done <= 1; state <= MASTER_ACK;
                        end
                    end
                end
                MASTER_ACK: begin
                    sda_en <= 1;
                    sda_out <= (b_cnt < bytes_to_read) ? 1'b0 : 1'b1;
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 12) i2c_scl <= 0;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0;
                        if (b_cnt < bytes_to_read) begin bit_cnt <= 7; state <= READ; end
                        else state <= STOP;
                    end
                end
                STOP: begin
                    sda_en <= 1; sda_out <= 0;
                    if (clk_cnt == 5) i2c_scl <= 1;
                    if (clk_cnt == 10) sda_out <= 1;
                    if (clk_cnt >= 15) begin
                        clk_cnt <= 0; state <= IDLE; ready <= 1;
                        b_cnt <= 0; bit_cnt <= 7;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule