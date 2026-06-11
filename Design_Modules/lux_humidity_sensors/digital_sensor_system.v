module digital_sensor_system #(
    parameter SIM_SPEEDUP = 1 
)(
    input  wire        clk, rst,
    inout  wire        sda, scl,
    output reg [15:0]  final_lux,
    output wire [15:0] final_hum    // <--- CHANGED TO WIRE
);

    localparam IDLE=0, READ_LUX=1, READ_HUM=2, WAIT=3;
    reg [1:0] state;
    
    reg start_i2c;
    reg [6:0] s_addr;
    reg [7:0] r_addr;
    wire [15:0] i2c_data;
    wire i2c_done;

    reg [21:0] t_cnt;
    wire [21:0] limit = (SIM_SPEEDUP) ? 2000 : 2000000;

    // --- CONTINUOUS BRUTE-FORCE MATH ---
    reg [15:0] data_buffer;
    // We use 33 bits to ensure no overflow on (65535 * 100)
    assign final_hum = (33'd100 * data_buffer) >> 16; 

    i2c_master_engine i2c_inst (
        .clk(clk), .rst(rst), .start(start_i2c),
        .slave_addr(s_addr), .reg_addr(r_addr),
        .read_data(i2c_data), .done(i2c_done),
        .sda(sda), .scl(scl)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            state <= IDLE; t_cnt <= 0; start_i2c <= 0; 
            final_lux <= 0; data_buffer <= 0;
        end else begin
            case (state)
                IDLE: begin
                    if (t_cnt >= limit) begin t_cnt <= 0; state <= READ_LUX; end
                    else t_cnt <= t_cnt + 1;
                end

                READ_LUX: begin
                    s_addr <= 7'h23; r_addr <= 8'h10;
                    start_i2c <= 1;
                    if (i2c_done) begin 
                        final_lux <= (i2c_data * 10) / 12; 
                        start_i2c <= 0;
                        state <= WAIT; 
                    end
                end

                WAIT: begin
                    if (t_cnt >= limit) begin t_cnt <= 0; state <= READ_HUM; end
                    else t_cnt <= t_cnt + 1;
                end

                READ_HUM: begin
                    s_addr <= 7'h40; r_addr <= 8'h01; 
                    start_i2c <= 1;
                    if (i2c_done) begin 
                        data_buffer <= i2c_data; // MATH UPDATES INSTANTLY VIA WIRE
                        start_i2c <= 0;
                        state <= IDLE;
                    end
                end
            endcase
        end
    end
endmodule