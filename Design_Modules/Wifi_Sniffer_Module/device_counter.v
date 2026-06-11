// =================================================================================
// Module Name:  device_counter
// Description:  Maintains a table of unique MAC addresses and counts them, 
//               applying an RSSI threshold filter to ignore distant devices.
// =================================================================================

module device_counter #(
    // --- Parameters Configuration ---
    parameter MAC_BYTES      = 6,   // Length of MAC Address in bytes
    parameter MAX_DEVICES    = 16,  // Maximum capacity of the internal lookup table
    parameter RSSI_THRESHOLD = -70  // Minimum signal strength required (e.g., -50 is OK, -80 is Ignored)
)(
    // --- Input Ports ---
    input  wire                   clk,          
    input  wire                   rst_n,        // Active-low asynchronous reset

    input  wire [8*MAC_BYTES-1:0] mac_in,
    input  wire signed [7:0]      rssi_in,      
    input  wire                   packet_valid, 
    input  wire                   packet_error, 

    // --- Output Ports ---
    output reg [7:0]              device_count, 
    output reg                    busy          
);

    // --- Internal Memory and Registers ---
    reg [8*MAC_BYTES-1:0] mac_table [0:MAX_DEVICES-1]; 
    reg [7:0]             stored_count;                

    integer i;       
    reg     match;   // Flag to indicate if the current MAC exists in the table

    // -----------------------------------------------------------------------------
    // Main Sequential Logic
    // -----------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // --- Reset State: Clear table and counters ---
            stored_count <= 0;
            device_count <= 0;
            busy         <= 0;
            for (i = 0; i < MAX_DEVICES; i = i + 1) begin
                mac_table[i] <= 0;
            end
        end else begin
            // Default State
            busy <= 0;
            
            // --- Packet Processing Logic ---
            // Triggered only if packet is valid, no errors, and signal is strong enough
            if (packet_valid && !packet_error && (rssi_in >= RSSI_THRESHOLD)) begin
                
                // 1. Check if MAC already exists in the table
                // Using blocking assignment for 'match' to get immediate result
                match = 0;
                for (i = 0; i < stored_count; i = i + 1) begin
                    if (mac_table[i] == mac_in) begin
                        match = 1;
                    end
                end

                // 2. If it's a NEW device and table has space, STORE it
                if (!match && (stored_count < MAX_DEVICES)) begin
                    mac_table[stored_count] <= mac_in;        // Add MAC to table
                    stored_count            <= stored_count + 1; // Increment internal count
                    device_count            <= stored_count + 1; // Update external output immediately
                end
                
                // Signal that we are processing/done with the current packet
                busy <= 1;
            end
        end
    end

endmodule
