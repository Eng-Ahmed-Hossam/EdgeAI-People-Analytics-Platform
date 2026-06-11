// =================================================================================
// Module Name:  WIFI_COUNTER_TOP
// Description:  The main top-level module that integrates UART, FIFO, Packet Parser, 
//               and the Device Counter with RSSI filtering.
// =================================================================================

module WIFI_COUNTER_TOP #(
    // --- Parameters Configuration ---
    parameter MAC_BYTES   = 6,      // Number of bytes for the MAC Address (Standard is 6)
    parameter RSSI_BYTES  = 3,      // Number of bytes reserved for RSSI data in packet
    parameter UART_DW     = 8,      // UART Data Width (Standard 8-bit)
    parameter RSSI_LIMIT  = -75     // Signal strength threshold (Ignore devices weaker than this)
)(
    // --- Input Ports ---
    input  wire       RST,          // Global Reset signal
    input  wire       RX_CLK,       // UART Receiver clock (Primary system clock)
    input  wire       SYS_CLK,      // Kept for compatibility (not actively used in this version)
    input  wire       RX_IN_S,      // Serial input from the Wi-Fi module (UART RX)
    
    // --- UART Configuration Ports ---
    input  wire [5:0] Prescale,     // Prescale value for Baud Rate generation
    input  wire       PAR_EN,       // Parity Enable bit
    input  wire       PAR_TYP,      // Parity Type (Even/Odd)

    // --- Output Ports ---
    output wire [7:0] device_count  // Total number of unique devices detected
);

    // -----------------------------------------------------------------------------
    // 1. Internal Wires for UART RX
    // -----------------------------------------------------------------------------
    wire [UART_DW-1:0] RX_OUT_P;     // Parallel data output from UART RX
    wire               RX_OUT_V;     // Valid signal: High when a full byte is received
    wire               Parity_Err;   // High if a parity error occurs
    wire               Stop_Err;     // High if a stop bit error occurs

    // --- UART Receiver Instance ---
    UART_RX #(
        .DWIDTH(UART_DW)
    ) u_uart_rx (
        .CLK(RX_CLK),
        .RST(RST),
        .RX_IN(RX_IN_S),
        .prescale(Prescale),
        .PAR_EN(PAR_EN),
        .PAR_TYP(PAR_TYP),
        .P_DATA(RX_OUT_P),
        .data_valid(RX_OUT_V),
        .Parity_Error(Parity_Err),
        .Stop_Error(Stop_Err)
    );

    // -----------------------------------------------------------------------------
    // 2. Internal Wires for FIFO 1 (Buffer)
    // -----------------------------------------------------------------------------
    wire [UART_DW-1:0] fifo1_data;   // Data output from the FIFO
    wire               fifo1_empty;  // High if FIFO has no more data

    // --- ASYNC FIFO Instance (Used here to buffer byte streams) ---
    ASYNC_FIFO #(
        .DATA_WIDTH(UART_DW)
    ) u_fifo1 (
        .W_CLK(RX_CLK),
        .W_RST(RST),
        .W_INC(RX_OUT_V & ~Parity_Err), // Write only when UART data is valid and no error
        .R_CLK(RX_CLK),
        .R_RST(RST),
        .R_INC(~fifo1_empty),           // Auto-read whenever data is available
        .WR_DATA(RX_OUT_P),
        .FULL(),                        // Full flag (not monitored in top)
        .EMPTY(fifo1_empty),
        .RD_DATA(fifo1_data)
    );

    // -----------------------------------------------------------------------------
    // 3. Internal Wires for Packet Parser
    // -----------------------------------------------------------------------------
    wire [8*MAC_BYTES-1:0] mac_val;  // Extracted MAC address (48-bit)
    wire signed [7:0]      rssi_val; // Extracted RSSI value (Signed integer)
    wire                   pkt_valid;// Pulse high when a full packet is parsed correctly
    wire                   pkt_error;// High if the packet format is invalid

    // --- Packet Parser Instance ---
    packet_parser #(
        .MAC_BYTES(MAC_BYTES),
        .RSSI_BYTES(RSSI_BYTES)
    ) u_parser (
        .clk(RX_CLK),
        .rst_n(RST),
        .data_in(fifo1_data),
        .data_valid(~fifo1_empty),
        .mac_out(mac_val),
        .rssi_out(rssi_val),
        .packet_valid(pkt_valid),
        .packet_error(pkt_error)
    );

    // -----------------------------------------------------------------------------
    // 4. Device Counter Instance
    // -----------------------------------------------------------------------------
    device_counter #(
        .MAC_BYTES(MAC_BYTES),
        .MAX_DEVICES(16),            // Capacity of the MAC lookup table
        .RSSI_THRESHOLD(RSSI_LIMIT)  // Filters out devices based on signal strength
    ) u_counter (
        .clk(RX_CLK),
        .rst_n(RST),
        .mac_in(mac_val),
        .rssi_in(rssi_val),
        .packet_valid(pkt_valid),
        .packet_error(pkt_error),
        .device_count(device_count),
        .busy()                      // Busy signal (not used at top level)
    );

endmodule
