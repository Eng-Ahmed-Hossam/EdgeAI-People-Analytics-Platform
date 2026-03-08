module ADT7420_Driver (
    input clk, reset,
    output reg [15:0] temperature, // Complete temperature value storage
    output reg data_valid,
    
    // Connection with the custom I2C Master
    output reg i2c_enable,
    output reg i2c_rw,
    output reg [7:0] i2c_data_in,
    input [7:0] i2c_data_out,
    input i2c_ready
);

    reg [2:0] state = 0;
    localparam IDLE = 0, ADDR_REG = 1, READ_MSB = 2, READ_LSB = 3, DONE = 4;

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            i2c_enable <= 0;
            temperature <= 0;
            data_valid <= 0;
        end else begin
            case(state)
                IDLE: begin
                    data_valid <= 0;
                    if(i2c_ready) begin
                        i2c_data_in <= 8'h00; // Temperature register address inside the sensor
                        i2c_rw <= 0;           // Write command to select the register
                        i2c_enable <= 1;
                        state <= ADDR_REG;
                    end
                end
                
                ADDR_REG: begin
                    if(!i2c_ready) i2c_enable <= 0; // Wait for Master to start
                    if(i2c_ready && !i2c_enable) begin
                        i2c_rw <= 1;           // Now request a read
                        i2c_enable <= 1;
                        state <= READ_MSB;
                    end
                end

                READ_MSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        temperature[15:8] <= i2c_data_out; // Store the first byte
                        i2c_enable <= 1;      // Request the second byte
                        state <= READ_LSB;
                    end
                end

                READ_LSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        temperature[7:0] <= i2c_data_out;  // Store the second byte
                        state <= DONE;
                    end
                end

                DONE: begin
                    data_valid <= 1;
                    state <= IDLE; // Continuous polling
                end
            endcase
        end
    end
endmodule