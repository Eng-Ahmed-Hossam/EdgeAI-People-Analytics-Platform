module i2c_master(
    input clk, reset,
    input [6:0] addr,       // Sensor address
    input [7:0] data_in,    // Data to be sent (to Register)
    input enable, rw,       // rw: 0 for Write, 1 for Read
    output reg [7:0] data_out = 8'h00,
    output reg ready = 1, 
    output reg ack_error = 0,
    output reg i2c_scl = 1,
    inout i2c_sda
);

    // Timing calculation: assuming clk=100MHz, we want SCL=100KHz
    // We need to divide the clock into four stages for each SCL pulse
    parameter DIVIDER = 250; 
    reg [15:0] count = 0;
    reg scl_clk = 0; // Slow internal clock for state control

    reg [3:0] state = 0;
    reg [3:0] bit_cnt = 7;
    reg sda_out = 1;
    reg sda_en = 1;

    // States
    localparam IDLE=0, START=1, ADDR=2, ACK1=3, WRITE=4, READ=5, ACK2=6, STOP=7;

    assign i2c_sda = (sda_en) ? sda_out : 1'bz;

    // Clock Divider generation
    always @(posedge clk) begin
        if (count == (DIVIDER/4) - 1) begin
            count <= 0;
            scl_clk <= ~scl_clk;
        end else begin
            count <= count + 1;
        end
    end

    // Main State Machine operating with the divided clock
    always @(posedge scl_clk or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            ready <= 1;
            i2c_scl <= 1;
            sda_out <= 1;
            sda_en <= 1;
            data_out <= 8'h00;
        end else begin
            case(state)
                IDLE: begin
                    ready <= 1;
                    i2c_scl <= 1;
                    sda_out <= 1;
                    sda_en <= 1;
                    if(enable) begin 
                        state <= START; 
                        ready <= 0; 
                    end
                end

                START: begin
                    sda_out <= 0; // SDA goes low first
                    i2c_scl <= 1; 
                    state <= ADDR;
                    bit_cnt <= 7;
                end

                ADDR: begin
                    i2c_scl <= ~i2c_scl; // Toggle clock
                    if (i2c_scl == 1) begin // Change data on falling edge
                        if (bit_cnt >= 1) begin
                            sda_out <= addr[bit_cnt-1];
                            bit_cnt <= bit_cnt - 1;
                        end else begin
                            sda_out <= rw;
                            state <= ACK1;
                        end
                    end
                end

                ACK1: begin
                    i2c_scl <= ~i2c_scl;
                    sda_en <= 0; // Release SDA for sensor to send ACK
                    if (i2c_scl == 1) begin
                        state <= (rw) ? READ : WRITE;
                        bit_cnt <= 7;
                    end
                end

                WRITE: begin
                    i2c_scl <= ~i2c_scl;
                    sda_en <= 1;
                    if (i2c_scl == 1) begin
                        sda_out <= data_in[bit_cnt];
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else state <= ACK2;
                    end
                end

                READ: begin
                    i2c_scl <= ~i2c_scl;
                    sda_en <= 0; // Receiving data
                    if (i2c_scl == 0) begin // Read while SCL is high
                        data_out[bit_cnt] <= i2c_sda;
                        if (bit_cnt > 0) bit_cnt <= bit_cnt - 1;
                        else state <= ACK2;
                    end
                end

                ACK2: begin
                    i2c_scl <= ~i2c_scl;
                    sda_en <= (rw) ? 1 : 0; // Master sends NACK in Read or receives ACK in Write
                    sda_out <= 1; // NACK
                    if (i2c_scl == 1) state <= STOP;
                end

                STOP: begin
                    i2c_scl <= ~i2c_scl;
                    if (i2c_scl == 0) begin
                        sda_en <= 1;
                        sda_out <= 0;
                    end else begin
                        sda_out <= 1; // Pulling SDA high while SCL is high = STOP
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule