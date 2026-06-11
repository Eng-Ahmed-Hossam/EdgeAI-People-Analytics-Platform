module i2c_master_engine (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [6:0]  slave_addr,
    input  wire [7:0]  reg_addr,     // The Register Pointer (0x00 or 0x01)
    output reg  [15:0] read_data,
    output reg         busy,
    output reg         done,
    inout  wire        sda,
    inout  wire        scl
);
    // Clock Timing (100kHz I2C from 100MHz System Clock)
    localparam DIV = 250; 
    reg [7:0] cnt;
    reg scl_clk, sda_out, scl_en;
    reg [3:0] state, bit_idx;
    reg [7:0] shift;

    // Physical Line Control (Open-Drain Simulation)
    assign sda = (sda_out) ? 1'bz : 1'b0;
    assign scl = (scl_en && !scl_clk) ? 1'b0 : 1'bz;

    // FSM States for Combined Format
    localparam S_IDLE    = 0, S_START   = 1, S_WR_ADDR = 2, 
               S_WR_REG  = 3, S_RESTART = 4, S_RD_ADDR = 5,
               S_RD_MSB  = 6, S_ACK     = 7, S_RD_LSB  = 8, 
               S_STOP    = 9;

    always @(posedge clk) begin
        if (busy) begin
            if (cnt == DIV-1) begin cnt <= 0; scl_clk <= ~scl_clk; end
            else cnt <= cnt + 1;
        end else begin cnt <= 0; scl_clk <= 1; end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE; busy <= 0; done <= 0; sda_out <= 1; scl_en <= 0;
        end else begin
            case (state)
                S_IDLE: begin
                    done <= 0;
                    if (start) begin busy <= 1; scl_en <= 1; state <= S_START; end
                end

                // 1. Send START and Slave Addr + WRITE bit
                S_START: if (scl_clk) begin 
                    sda_out <= 0; state <= S_WR_ADDR; bit_idx <= 7; shift <= {slave_addr, 1'b0}; 
                end

                S_WR_ADDR: if (!scl_clk && cnt == 0) begin
                    sda_out <= shift[bit_idx];
                    if (bit_idx == 0) state <= S_WR_REG; else bit_idx <= bit_idx - 1;
                end

                // 2. Send the Register Address (the pointer)
                S_WR_REG: if (scl_clk && cnt == DIV/2) begin
                    shift <= reg_addr; bit_idx <= 7; state <= S_RESTART;
                end else if (!scl_clk && cnt == 0) begin
                    sda_out <= shift[bit_idx];
                    if (bit_idx != 0) bit_idx <= bit_idx - 1;
                end

                // 3. Repeated START for the Read Phase
                S_RESTART: if (scl_clk) begin
                    sda_out <= 1; // Pull High
                    if (cnt == DIV-1) state <= S_RD_ADDR;
                end

                S_RD_ADDR: if (scl_clk && sda_out == 1) begin
                    sda_out <= 0; // Repeated Start
                    shift <= {slave_addr, 1'b1}; bit_idx <= 7; state <= S_RD_MSB;
                end

                // 4. Read the 16-bit data (MSB then LSB)
                S_RD_MSB: if (scl_clk && cnt == DIV/2) begin
                    sda_out <= 1; // Release line to read
                    read_data[bit_idx + 8] <= sda;
                    if (bit_idx == 0) state <= S_ACK; else bit_idx <= bit_idx - 1;
                end

                S_ACK: if (!scl_clk && cnt == 0) begin
                    sda_out <= 0; // Master ACK
                    state <= S_RD_LSB; bit_idx <= 7;
                end

                S_RD_LSB: if (scl_clk && cnt == DIV/2) begin
                    sda_out <= 1; // Release line
                    read_data[bit_idx] <= sda;
                    if (bit_idx == 0) state <= S_STOP; else bit_idx <= bit_idx - 1;
                end

                S_STOP: if (scl_clk) begin
                    sda_out <= 1; busy <= 0; done <= 1; state <= S_IDLE;
                end
            endcase
        end
    end
endmodule