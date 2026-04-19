// =================================================================================
// Module Name:  tb_WIFI_COUNTER_TOP
// Description:  Comprehensive Testbench to verify RSSI Filtering and MAC Duplicate
//               Detection in the WIFI_COUNTER_TOP system.
// =================================================================================

`timescale 1ns/1ps

module tb_WIFI_COUNTER_TOP;

    // --- Testbench Parameters ---
    parameter MAC_BYTES = 6;

    // --- Internal Signals (Stimulus) ---
    reg        RX_CLK;       // Clock signal for UART RX domain
    reg        RST;          // System Reset
    reg        RX_IN;        // UART Serial Input bit
    reg [5:0]  Prescale;     // UART Prescale configuration

    // --- Observed Signals (Outputs) ---
    wire [7:0] device_count; // Number of unique devices detected by DUT

    // -----------------------------------------------------------------------------
    // Clock Generation
    // -----------------------------------------------------------------------------
    real RX_CLK_PERIOD = 542.59; // Clock period for the simulation
    initial begin
        RX_CLK = 0;
        forever #(RX_CLK_PERIOD/2) RX_CLK = ~RX_CLK;
    end

    // -----------------------------------------------------------------------------
    // Device Under Test (DUT) Instance
    // -----------------------------------------------------------------------------
    WIFI_COUNTER_TOP #(
        .RSSI_LIMIT(-75)     // Set filter to ignore signals weaker than -75 dBm
    ) uut (
        .RST(RST),
        .RX_CLK(RX_CLK),
        .SYS_CLK(RX_CLK),    // Shared clock for simplicity
        .RX_IN_S(RX_IN),
        .Prescale(6'd16),
        .PAR_EN(1'b0),
        .PAR_TYP(1'b0),
        .device_count(device_count)
    );

    // -----------------------------------------------------------------------------
    // Task: send_byte
    // Description: Simulates a UART frame (Start bit, 8 data bits, Stop bit).
    // -----------------------------------------------------------------------------
    task send_byte(input [7:0] data);
        integer i;
        begin
            RX_IN = 0; // --- Start Bit ---
            #(RX_CLK_PERIOD * 16);
            
            // --- Data Bits (LSB First) ---
            for (i = 0; i < 8; i = i + 1) begin
                RX_IN = data[i];
                #(RX_CLK_PERIOD * 16);
            end
            
            RX_IN = 1; // --- Stop Bit ---
            #(RX_CLK_PERIOD * 16);
        end
    endtask

    // -----------------------------------------------------------------------------
    // Task: send_packet
    // Description: Sends a full Wi-Fi packet (MAC Address + Comma + RSSI + Newline).
    // -----------------------------------------------------------------------------
    task send_packet(input [8*MAC_BYTES-1:0] mac, input signed [7:0] rssi);
        integer i;
        begin
            $display("--- Sending MAC: %s | RSSI: %0d ---", mac, rssi);
            
            // 1. Send MAC Address (Character by Character)
            for (i = MAC_BYTES - 1; i >= 0; i = i - 1) begin
                send_byte(mac[8*i +: 8]);
            end
            
            // 2. Send Delimiter (Comma ',')
            send_byte(8'd44);
            
            // 3. Send Dummy RSSI characters (The 'force' below handles actual logic)
            send_byte(8'd54); // '6'
            send_byte(8'd54); // '6'
            
            // 4. Send Packet Terminator (Newline '\n')
            send_byte(8'd10);
        end
    endtask

    // -----------------------------------------------------------------------------
    // Main Test Sequence
    // -----------------------------------------------------------------------------
    initial begin
        // --- System Reset Sequence ---
        RX_IN = 1;
        RST   = 1; // High (Assuming Active High reset for logic initialization)
        #(RX_CLK_PERIOD * 32);
        RST   = 0; // Reset pulse
        #(RX_CLK_PERIOD * 32);
        RST   = 1; // Release reset
        #(RX_CLK_PERIOD * 100);

        // --- [Test Case 1]: New Device + Strong Signal (-50 dBm) ---
        // Expected Outcome: Device count increases to 1.
        force uut.u_parser.rssi_out = -50; 
        send_packet("DEVICE1", -50);
        #(RX_CLK_PERIOD * 2000);

        // --- [Test Case 2]: New Device + Weak Signal (-90 dBm) ---
        // Expected Outcome: Device count stays 1 (Filtered out).
        release uut.u_parser.rssi_out;
        force uut.u_parser.rssi_out = -90;
        send_packet("DEVICE2", -90);
        #(RX_CLK_PERIOD * 2000);

        // --- [Test Case 3]: Duplicate Device (DEVICE1) + Strong Signal ---
        // Expected Outcome: Device count stays 1 (Duplicate detected).
        release uut.u_parser.rssi_out;
        force uut.u_parser.rssi_out = -40;
        send_packet("DEVICE1", -40); 
        #(RX_CLK_PERIOD * 2000);

        // --- [Test Case 4]: New Device + Strong Signal (-30 dBm) ---
        // Expected Outcome: Device count increases to 2.
        release uut.u_parser.rssi_out;
        force uut.u_parser.rssi_out = -30;
        send_packet("DEVICE3", -30);
        #(RX_CLK_PERIOD * 2000);

        // --- Final Test Result Reporting ---
        $display("=======================================");
        $display("FINAL DEVICE COUNT: %0d", device_count);
        if (device_count == 2) 
            $display("TEST PASSED: Filter & Duplicate logic are working!");
        else 
            $display("TEST FAILED: Logic error detected!");
        $display("=======================================");
        
        $stop; // End Simulation
    end

    // -----------------------------------------------------------------------------
    // Simulation Monitor
    // -----------------------------------------------------------------------------
    always @(posedge RX_CLK) begin
        if (uut.u_counter.packet_valid) begin
            $display("Time=%0t | Packet Parsed: RSSI=%0d | Threshold=%0d", 
                      $time, uut.u_counter.rssi_in, uut.u_counter.RSSI_THRESHOLD);
        end
    end

endmodule