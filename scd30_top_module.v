module scd30_top(
    input clk, reset_n, read_btn,
    output [15:0] co2_leds,
    output done_tick,
    output i2c_scl,
    inout i2c_sda
);
    wire reset = !reset_n;
    wire [6:0] w_addr;
    wire [7:0] w_data_in, w_data_out;
    wire w_enable, w_rw, w_ready;

    i2c_master master_inst (
        .clk(clk), .reset(reset), .addr(w_addr), .data_in(w_data_in),
        .enable(w_enable), .rw(w_rw), .data_out(w_data_out),
        .ready(w_ready), .i2c_scl(i2c_scl), .i2c_sda(i2c_sda)
    );

    scd30_co2_driver driver_inst (
        .clk(clk), .reset(reset), .start(read_btn),
        .co2_data(co2_leds), .done(done_tick),
        .i2c_addr(w_addr), .i2c_data_in(w_data_in),
        .i2c_enable(w_enable), .i2c_rw(w_rw),
        .i2c_ready(w_ready), .i2c_data_out(w_data_out)
    );
endmodule