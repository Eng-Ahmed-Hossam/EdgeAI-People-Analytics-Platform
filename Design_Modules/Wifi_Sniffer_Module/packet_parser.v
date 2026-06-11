//==============================================================
// Packet Parser RTL - FIXED VERSION
// Format: <MAC>,<RSSI>\n
//==============================================================
module packet_parser #(
    parameter MAC_BYTES  = 6,
    parameter RSSI_BYTES = 3
)(
    input  wire              clk,
    input  wire              rst_n,
    input  wire [7:0]        data_in,
    input  wire              data_valid,

    output reg  [8*MAC_BYTES-1:0]  mac_out,
    output reg  signed [7:0]       rssi_out,
    output reg                     packet_valid,
    output reg                     packet_error
);

    // FSM
    reg [1:0] state;
    localparam IDLE      = 2'b00;
    localparam READ_MAC  = 2'b01;
    localparam READ_RSSI = 2'b10;

    // Buffers
    reg [8*MAC_BYTES-1:0]  mac_buffer;
    reg [8*RSSI_BYTES-1:0] rssi_buffer;

    integer mac_cnt;
    integer rssi_cnt;
    integer i;

    reg signed [7:0] rssi_tmp;
    reg rssi_negative;

    //==========================================================
    // FSM
    //==========================================================
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state        <= IDLE;
            mac_buffer   <= 0;
            rssi_buffer  <= 0;
            mac_cnt      <= 0;
            rssi_cnt     <= 0;
            packet_valid <= 0;
            packet_error <= 0;
            mac_out      <= 0;
            rssi_out     <= 0;
        end else begin

            // default
            packet_valid <= 0;
            packet_error <= 0;

            if(data_valid) begin
                case(state)

                //=====================
                // IDLE
                //=====================
                IDLE: begin
                    mac_buffer  <= 0;
                    rssi_buffer <= 0;
                    mac_cnt     <= 0;
                    rssi_cnt    <= 0;

                    if(data_in != 8'h0A && data_in != 8'h0D) begin
                        state <= READ_MAC;
                        mac_buffer[8*(MAC_BYTES-1) +:8] <= data_in;
                        mac_cnt <= 1;
                    end
                end

                //=====================
                // READ MAC
                //=====================
                READ_MAC: begin
                    if(data_in == 8'd44) begin // ','
                        state <= READ_RSSI;
                    end
                    else if(mac_cnt < MAC_BYTES) begin
                        mac_buffer[8*(MAC_BYTES-1-mac_cnt) +:8] <= data_in;
                        mac_cnt <= mac_cnt + 1;
                    end
                end

                //=====================
                // READ RSSI
                //=====================
                READ_RSSI: begin
                    if(data_in == 8'd10 || data_in == 8'd13) begin // \n or \r

                        rssi_tmp      <= 0;
                        rssi_negative <= 0;

                        if(rssi_cnt == 0) begin
                            packet_error <= 1;
                        end else begin

                            // check sign
                            if(rssi_buffer[8*(RSSI_BYTES-1) +:8] == 8'd45) begin
                                rssi_negative <= 1;
                            end

                            // convert digits
                            for(i = 0; i < rssi_cnt; i = i + 1) begin
                                if(rssi_buffer[8*(RSSI_BYTES-1-i) +:8] >= 8'd48 &&
                                   rssi_buffer[8*(RSSI_BYTES-1-i) +:8] <= 8'd57) begin

                                    rssi_tmp <= (rssi_tmp * 10) +
                                       (rssi_buffer[8*(RSSI_BYTES-1-i) +:8] - 8'd48);
                                end
                                else if(rssi_buffer[8*(RSSI_BYTES-1-i) +:8] != 8'd45) begin
                                    packet_error <= 1;
                                end
                            end

                            if(rssi_negative)
                                rssi_out <= -rssi_tmp;
                            else
                                rssi_out <= rssi_tmp;
                        end

                        mac_out      <= mac_buffer;
                        packet_valid <= 1;
                        state        <= IDLE;
                    end
                    else if(rssi_cnt < RSSI_BYTES) begin
                        rssi_buffer[8*(RSSI_BYTES-1-rssi_cnt) +:8] <= data_in;
                        rssi_cnt <= rssi_cnt + 1;
                    end
                end

                default: state <= IDLE;

                endcase
            end
        end
    end

endmodule