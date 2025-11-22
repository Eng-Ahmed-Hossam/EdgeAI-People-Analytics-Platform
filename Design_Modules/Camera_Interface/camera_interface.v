`timescale 1ns / 1ps

/*
 * camera_interface.v
 *
 * Interface module for OV7670 camera.
 * Handles SCCB/I2C configuration, retrieves pixel data, and writes to an asynchronous FIFO.
 *
 * Inputs:
 * - clk, clk_100  : system clocks
 * - rst_n         : active-low reset
 * - key[3:0]      : push-buttons for brightness/contrast control
 * - rd_en         : FIFO read enable
 * - cmos_pclk     : camera pixel clock
 * - cmos_href     : horizontal reference
 * - cmos_vsync    : vertical sync
 * - cmos_db[7:0]  : camera data bus
 *
 * Outputs:
 * - data_count_r  : FIFO data count
 * - dout[15:0]   : FIFO output pixel data
 * - cmos_sda, cmos_scl : I2C wires
 * - cmos_rst_n, cmos_pwdn, cmos_xclk : camera control signals
 * - led[3:0]     : debug LEDs
 */

module camera_interface(
    input  wire clk,
    input  wire clk_100,
    input  wire rst_n,
    input  wire [3:0] key, // key[1:0] brightness, key[3:2] contrast
    input  wire rd_en,
    output wire [9:0] data_count_r,
    output wire [15:0] dout,
    input  wire cmos_pclk,
    input  wire cmos_href,
    input  wire cmos_vsync,
    input  wire [7:0] cmos_db,
    inout  wire cmos_sda,
    inout  wire cmos_scl,
    output wire cmos_rst_n,
    output wire cmos_pwdn,
    output wire cmos_xclk,
    output wire [3:0] led
);

    // ----------------------
    // FSM state declarations
    // ----------------------
    localparam IDLE           = 4'd0,
               START_SCCB     = 4'd1,
               WRITE_ADDRESS  = 4'd2,
               WRITE_DATA     = 4'd3,
               DIGEST_LOOP    = 4'd4,
               DELAY          = 4'd5,
               VSYNC_FEDGE    = 4'd6,
               BYTE1          = 4'd7,
               BYTE2          = 4'd8,
               FIFO_WRITE     = 4'd9,
               STOPPING       = 4'd10;

    localparam WAIT_INIT  = 3'd0,
               SCCB_IDLE  = 3'd1,
               SCCB_ADDR  = 3'd2,
               SCCB_DATA  = 3'd3,
               SCCB_STOP  = 3'd4;

    localparam MSG_INDEX = 8'd77; // last SCCB message index

    // FSM registers
    reg [3:0] state_q = IDLE, state_d;
    reg [2:0] sccb_state_q = WAIT_INIT, sccb_state_d;
    reg [7:0] addr_q=0, addr_d;
    reg [7:0] data_q=0, data_d;
    reg [7:0] brightness_q=0, brightness_d;
    reg [7:0] contrast_q=0, contrast_d;
    reg [15:0] pixel_q=0, pixel_d;
    reg [7:0] message_index_q=0, message_index_d;
    reg [27:0] delay_q=0, delay_d;
    reg start_delay_q=0, start_delay_d;

    // SCCB/I2C signals
    reg start, stop;
    reg [7:0] wr_data;
    wire [1:0] ack;
    wire [7:0] rd_data;
    wire [3:0] state;

    // FIFO
    reg wr_en;
    wire full;

    // key debounced ticks
    wire key0_tick, key1_tick, key2_tick, key3_tick;

    // input buffers for synchronization
    reg pclk_1, pclk_2, href_1, href_2, vsync_1, vsync_2;

    // debug LEDs
    reg [3:0] led_q=0, led_d;

    // ----------------------
    // Camera initialization messages
    // ----------------------
    reg [15:0] message[0:250];

    initial begin
        message[0]  = 16'h1280; // reset camera
        message[1]  = 16'h1204; // RGB output
        message[2]  = 16'h1520; // pclk control
        message[3]  = 16'h40D0; // RGB565
        message[4]  = 16'h1204;
        message[5]  = 16'h1180;
        message[6]  = 16'h0C00;
        message[7]  = 16'h3E00;
        message[8]  = 16'h0400;
        message[9]  = 16'h40D0;
        message[10] = 16'h3A04;
        message[11] = 16'h1418;
        message[12] = 16'h4FB3;
        message[13] = 16'h50B3;
        message[14] = 16'h5100;
        message[15] = 16'h523D;
        message[16] = 16'h53A7;
        message[17] = 16'h54E4;
        message[18] = 16'h589E;
        message[19] = 16'h3DC0;
        message[20] = 16'h1714;
        message[21] = 16'h1802;
        message[22] = 16'h3280;
        message[23] = 16'h1903;
        message[24] = 16'h1A7B;
        message[25] = 16'h030A;
        message[26] = 16'h0F41;
        message[27] = 16'h1E00;
        message[28] = 16'h330B;
        message[29] = 16'h3C78;
        message[30] = 16'h6900;
        message[31] = 16'h7400;
        message[32] = 16'hB084;
        message[33] = 16'hB10C;
        message[34] = 16'hB20E;
        message[35] = 16'hB380;
        message[36] = 16'h703A;
        message[37] = 16'h7135;
        message[38] = 16'h7211;
        message[39] = 16'h73F0;
        message[40] = 16'hA202;
        message[41] = 16'h7A20;
        message[42] = 16'h7B10;
        message[43] = 16'h7C1E;
        message[44] = 16'h7D35;
        message[45] = 16'h7E5A;
        message[46] = 16'h7F69;
        message[47] = 16'h8076;
        message[48] = 16'h8180;
        message[49] = 16'h8288;
        message[50] = 16'h838F;
        message[51] = 16'h8496;
        message[52] = 16'h85A3;
        message[53] = 16'h86AF;
        message[54] = 16'h87C4;
        message[55] = 16'h88D7;
        message[56] = 16'h89E8;
        message[57] = 16'h13E0;
        message[58] = 16'h0000;
        message[59] = 16'h1000;
        message[60] = 16'h0D40;
        message[61] = 16'h1418;
        message[62] = 16'hA505;
        message[63] = 16'hAB07;
        message[64] = 16'h2495;
        message[65] = 16'h2533;
        message[66] = 16'h26E3;
        message[67] = 16'h9F78;
        message[68] = 16'hA068;
        message[69] = 16'hA103;
        message[70] = 16'hA6D8;
        message[71] = 16'hA7D8;
        message[72] = 16'hA8F0;
        message[73] = 16'hA990;
        message[74] = 16'hAA94;
        message[75] = 16'h13E5;
        message[76] = 16'h1E23;
        message[77] = 16'h6906;
    end

    // ----------------------
    // Register update on clock
    // ----------------------
    always @(posedge clk_100 or negedge rst_n) begin
        if (!rst_n) begin
            state_q <= IDLE;
            led_q <= 0;
            delay_q <= 0;
            start_delay_q <= 0;
            message_index_q <= 0;
            pixel_q <= 0;
            sccb_state_q <= WAIT_INIT;
            addr_q <= 0;
            data_q <= 0;
            brightness_q <= 0;
            contrast_q <= 0;
        end else begin
            state_q <= state_d;
            led_q <= led_d;
            delay_q <= delay_d;
            start_delay_q <= start_delay_d;
            message_index_q <= message_index_d;
            pixel_q <= pixel_d;
            sccb_state_q <= sccb_state_d;
            addr_q <= addr_d;
            data_q <= data_d;
            brightness_q <= brightness_d;
            contrast_q <= contrast_d;

            // synchronize camera signals
            pclk_1 <= cmos_pclk; pclk_2 <= pclk_1;
            href_1 <= cmos_href; href_2 <= href_1;
            vsync_1 <= cmos_vsync; vsync_2 <= vsync_1;
        end
    end

    // ----------------------
    // Assign fixed outputs
    // ----------------------
    assign cmos_pwdn = 0;
    assign cmos_rst_n = 1;
    assign led = led_q;

    // ----------------------
    // Module instantiations
    // ----------------------
    i2c_top #(.freq(100_000)) u_i2c (
        .clk(clk_100),
        .rst_n(rst_n),
        .start(start),
        .stop(stop),
        .wr_data(wr_data),
        .rd_tick(), // not used
        .ack(ack),
        .rd_data(rd_data),
        .scl(cmos_scl),
        .sda(cmos_sda),
        .state(state)
    );

    dcm_24MHz u_dcm (
        .clk(clk),
        .cmos_xclk(cmos_xclk),
        .RESET(~rst_n),
        .LOCKED() // optional
    );

    asyn_fifo #(.DATA_WIDTH(16), .FIFO_DEPTH_WIDTH(10)) u_fifo (
        .rst_n(rst_n),
        .clk_write(clk_100),
        .clk_read(clk_100),
        .write(wr_en),
        .read(rd_en),
        .data_write(pixel_q),
        .data_read(dout),
        .full(full),
        .empty(),
        .data_count_r(data_count_r)
    );

    // Debounced keys
    debounce_explicit u_key0 (.clk(clk_100), .rst_n(rst_n), .sw(!key[0]), .db_level(), .db_tick(key0_tick));
    debounce_explicit u_key1 (.clk(clk_100), .rst_n(rst_n), .sw(!key[1]), .db_level(), .db_tick(key1_tick));
    debounce_explicit u_key2 (.clk(clk_100), .rst_n(rst_n), .sw(!key[2]), .db_level(), .db_tick(key2_tick));
    debounce_explicit u_key3 (.clk(clk_100), .rst_n(rst_n), .sw(!key[3]), .db_level(), .db_tick(key3_tick));

endmodule
