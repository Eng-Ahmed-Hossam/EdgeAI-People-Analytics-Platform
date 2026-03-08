module i2c_master (
    input clk, reset,
    input [6:0] addr,
    input [7:0] data_in,
    input enable, rw,
    output reg [7:0] data_out,
    output reg ready,
    output reg i2c_scl,
    inout i2c_sda
);
    // State definitions
    localparam IDLE   = 3'd0;
    localparam START  = 3'd1;
    localparam ADDR   = 3'd2;
    localparam ACK1   = 3'd3;
    localparam WRITE  = 3'd4; // Added WRITE state for completeness
    localparam READ   = 3'd5;
    localparam NACK   = 3'd6;
    localparam STOP   = 3'd7;

    reg [2:0] state; // Sufficient for 8 states
    reg [3:0] bit_cnt;
    reg [6:0] clk_cnt; 
    reg sda_out, sda_en;

    assign i2c_sda = (sda_en) ? sda_out : 1'bz;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE; i2c_scl <= 1; ready <= 1; sda_en <= 0; clk_cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    ready <= 1; i2c_scl <= 1; sda_en <= 0; clk_cnt <= 0;
                    if (enable) begin 
                        state <= START; 
                        ready <= 0; 
                        sda_en <= 1; 
                        sda_out <= 0; 
                    end
                end

                START: begin
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 50) i2c_scl <= 0;
                    if (clk_cnt == 99) begin 
                        clk_cnt <= 0; 
                        state <= ADDR; 
                        bit_cnt <= 7; 
                    end
                end

                ADDR: begin
                    sda_en <= 1;
                    // Send Address then Read/Write bit
                    sda_out <= (bit_cnt >= 1) ? addr[bit_cnt-1] : rw;
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    if (clk_cnt == 75) i2c_scl <= 0;
                    if (clk_cnt == 99) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else begin state <= ACK1; sda_en <= 0; end 
                    end
                end

                ACK1: begin
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    if (clk_cnt == 75) i2c_scl <= 0;
                    if (clk_cnt == 99) begin 
                        clk_cnt <= 0; 
                        bit_cnt <= 7; 
                        state <= (rw) ? READ : WRITE; 
                    end
                end

                WRITE: begin // Optional state for sending sensor commands
                    sda_en <= 1;
                    sda_out <= data_in[bit_cnt];
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    if (clk_cnt == 75) i2c_scl <= 0;
                    if (clk_cnt == 99) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else state <= STOP; // Or transition to ACK2
                    end
                end

                READ: begin
                    sda_en <= 0;
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    // Sampling data in the middle of the pulse
                    if (clk_cnt == 50) data_out[bit_cnt] <= i2c_sda; 
                    if (clk_cnt == 75) i2c_scl <= 0;
                    if (clk_cnt == 99) begin
                        clk_cnt <= 0;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else state <= NACK;
                    end
                end

                NACK: begin // Master sends NACK after reading byte
                    sda_en <= 1; sda_out <= 1; 
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    if (clk_cnt == 75) i2c_scl <= 0;
                    if (clk_cnt == 99) begin clk_cnt <= 0; state <= STOP; end
                end

                STOP: begin
                    sda_en <= 1; sda_out <= 0;
                    clk_cnt <= clk_cnt + 1;
                    if (clk_cnt == 25) i2c_scl <= 1;
                    if (clk_cnt == 50) sda_out <= 1;
                    if (clk_cnt == 99) begin clk_cnt <= 0; state <= IDLE; end
                end

                default: state <= IDLE;
            endcase
        end
    end
endmodule