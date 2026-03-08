module scd30_co2_driver(
    input clk, reset, start,
    output reg [15:0] co2_data,
    output reg done,
    output reg [6:0] i2c_addr,
    output reg [7:0] i2c_data_in,
    output reg i2c_enable, i2c_rw,
    input i2c_ready,
    input [7:0] i2c_data_out
);
    reg [2:0] state;
    localparam IDLE=0, READ_MSB=1, WAIT_MSB=2, READ_LSB=3, WAIT_LSB=4, FINISH=5;

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            state <= IDLE; co2_data <= 0; done <= 0; i2c_enable <= 0;
        end else begin
            case(state)
                IDLE: begin
                    done <= 0;
                    if(start) begin
                        state <= READ_MSB;
                        i2c_addr <= 7'h61;
                        i2c_rw <= 1;
                    end
                end
                READ_MSB: begin
                    if(i2c_ready) begin i2c_enable <= 1; state <= WAIT_MSB; end
                end
                WAIT_MSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        co2_data[15:8] <= i2c_data_out;
                        state <= READ_LSB;
                    end
                end
                READ_LSB: begin
                    if(i2c_ready) begin i2c_enable <= 1; state <= WAIT_LSB; end
                end
                WAIT_LSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        co2_data[7:0] <= i2c_data_out;
                        state <= FINISH;
                    end
                end
                FINISH: begin
                    done <= 1;
                    state <= IDLE;
                end
            endcase
        end
    end
endmodule