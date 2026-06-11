/*
 * i2c_top.v
 *
 * I2C / SCCB Master Controller (Bit-Banged)
 *
 * This module implements a bit-banged master supporting START/STOP, byte
 * write/read (MSB first), ACK handling, repeated-starts, and configurable
 * SCL timing. It supports two modes:
 *   - SCCB (default, push-pull SCL/SDA behavior used by many camera modules)
 *   - I2C  (open-drain SCL/SDA; set parameter I2C_MODE = 1)
 *
 * Parameters:
 *  - freq     : desired SCL frequency (Hz). Default 100_000.
 *  - I2C_MODE : 0 => SCCB (default), 1 => true I2C open-drain behavior.
 *
 * Inputs:
 *  - clk, rst_n   : system clock and active-low reset
 *  - start, stop  : start / stop control inputs (start used also for repeated-start)
 *  - wr_data      : byte presented by master for transmission (address or data)
 *
 * Outputs:
 *  - rd_tick : 1-cycle pulse indicating rd_data is ready
 *  - ack     : {ack_tick, ack_value}  ack_tick = 1 when ACK bit sampled,
 *              ack_value = 1 for ACK (SDA low), 0 for NACK
 *  - rd_data : received data byte from slave
 *  - state   : FSM state (for debug)
 *
 * Inouts:
 *  - scl, sda : physical bus lines (driven according to mode)
 */

`timescale 1ns/1ps

module i2c_top #(
    parameter integer freq     = 100_000,
    parameter        I2C_MODE = 0   // 0 = SCCB (push-pull), 1 = I2C (open-drain)
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,
    input  wire        stop,
    input  wire [7:0]  wr_data,

    output reg         rd_tick,    // registered 1-cycle pulse when rd_data ready
    output reg  [1:0]  ack,        // ack[1]=tick (1-cycle), ack[0]=ACK(1)/NACK(0) value (registered)
    output wire [7:0]  rd_data,

    inout              scl,
    inout              sda,

    output wire [3:0]  state
);

    // -------------------------
    // Timing constants / helpers
    // -------------------------
    localparam integer full  = (100_000_000) / (2 * freq); // counts per half SCL period
    localparam integer half  = full / 2;
    localparam integer counter_width = log2(full);

    function integer log2(input integer n);
        integer i;
        begin
            log2 = 1;
            for (i = 0; 2**i < n; i = i + 1)
                log2 = i + 1;
        end
    endfunction

    // -------------------------
    // FSM states
    // -------------------------
    localparam [3:0]
        IDLE        = 0,
        STARTING    = 1,
        PACKET      = 2,
        ACK_SERVANT = 3,
        RENEW_DATA  = 4,
        READ        = 5,
        ACK_MASTER  = 6,
        STOP_1      = 7,
        STOP_2      = 8;

    // -------------------------
    // Registers / wires
    // -------------------------
    reg [3:0]  state_q, state_d;
    reg        start_req_q, start_req_d;   // remembers repeated-start request
    reg        read_flag_q, read_flag_d;   // remembers R/W bit from loaded address
    reg [3:0]  idx_q, idx_d;
    reg [7:0]  wr_data_q, wr_data_d;       // shift register for transmit (8-bit)
    reg [7:0]  rd_data_q, rd_data_d;

    reg        scl_q, scl_d;
    reg        sda_q, sda_d;

    reg [counter_width-1:0] counter_q, counter_d;

    // Registered pulses/flags (internal)
    reg ack_tick_d, ack_val_d;
    reg rd_tick_d;

    // sampling edges derived from internal counters
    wire scl_hi; // sampling edge when SCL is high middle
    wire scl_lo; // sampling edge when SCL is low middle

    // -------------------------
    // Sequential registers (clocked)
    // -------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_q       <= IDLE;
            start_req_q   <= 0;
            read_flag_q   <= 0;
            idx_q         <= 0;
            wr_data_q     <= 0;
            rd_data_q     <= 0;
            scl_q         <= 1'b1;
            sda_q         <= 1'b1;
            counter_q     <= 0;
            // outputs
            rd_tick       <= 0;
            ack           <= 0;
        end else begin
            state_q       <= state_d;
            start_req_q   <= start_req_d;
            read_flag_q   <= read_flag_d;
            idx_q         <= idx_d;
            wr_data_q     <= wr_data_d;
            rd_data_q     <= rd_data_d;
            scl_q         <= scl_d;
            sda_q         <= sda_d;
            counter_q     <= counter_d;

            // register pulses/outputs (one-cycle pulses driven by internal flags)
            rd_tick       <= rd_tick_d;
            ack[1]        <= ack_tick_d;
            ack[0]        <= ack_val_d;
        end
    end

    // -------------------------
    // Clock divider / SCL generation
    // - Do NOT run counter in IDLE/STARTING (prevents spurious toggles)
    // -------------------------
    always @(*) begin
        // defaults: hold values
        counter_d = counter_q;
        scl_d     = scl_q;

        if (state_q == IDLE || state_q == STARTING) begin
            // keep SCL high and reset counter in idle/start to avoid glitches
            scl_d     = 1'b1;
            counter_d = 0;
        end else begin
            // normal toggling operation
            if (counter_q >= full) begin
                counter_d = 0;
                scl_d     = ~scl_q;
            end else begin
                counter_d = counter_q + 1'b1;
                scl_d     = scl_q;
            end
        end
    end

    // sampling edges (based only on internal state)
    assign scl_hi = (scl_q == 1'b1 && counter_q == half);
    assign scl_lo = (scl_q == 1'b0 && counter_q == half);

    // -------------------------
    // FSM next-state and data-path (combinational)
    // - compute next values for registers and internal pulse flags
    // -------------------------
    always @(*) begin
        // defaults
        state_d      = state_q;
        start_req_d  = start_req_q;
        read_flag_d  = read_flag_q;
        idx_d        = idx_q;
        wr_data_d    = wr_data_q;
        rd_data_d    = rd_data_q;
        sda_d        = sda_q;

        // internal pulse flags default low
        ack_tick_d   = 1'b0;
        ack_val_d    = 1'b0;
        rd_tick_d    = 1'b0;

        case (state_q)

            // -----------------------------------------------------------------
            // IDLE: wait for start; capture R/W bit (read_flag) for address byte
            // -----------------------------------------------------------------
            IDLE: begin
                sda_d = 1'b1; // bus idle
                if (start) begin
                    // load transmit byte (address or first data)
                    wr_data_d   = wr_data;
                    read_flag_d = wr_data[0]; // R/W in LSB of address byte
                    start_req_d = start;      // capture request (used for repeated-start)
                    idx_d       = 4'd7;       // MSB first
                    state_d     = STARTING;
                end
            end

            // -----------------------------------------------------------------
            // STARTING: generate START (SDA goes low while SCL is high)
            // Wait until we are at SCL high sample point to assert SDA low.
            // -----------------------------------------------------------------
            STARTING: begin
                // we sample edge using scl_hi (SCL high middle). As soon as that
                // sampling edge arrives we assert SDA low to create START.
                if (scl_hi) begin
                    sda_d = 1'b0;
                    state_d = PACKET;
                end else begin
                    sda_d = 1'b1; // hold high before start moment
                end
            end

            // -----------------------------------------------------------------
            // PACKET: transmit 8 bits MSB first; output bits when SCL low middle
            // -----------------------------------------------------------------
            PACKET: begin
                // drive SDA stable during SCL low window; change on scl_lo
                if (scl_lo) begin
                    sda_d = wr_data_q[idx_q];
                    if (idx_q == 0) begin
                        idx_d = 0;
                        state_d = ACK_SERVANT;
                    end else begin
                        idx_d = idx_q - 1;
                    end
                end else begin
                    // keep the SDA as is between edges
                    sda_d = sda_q;
                end
            end

            // -----------------------------------------------------------------
            // ACK_SERVANT: release SDA, sample ACK from slave when SCL high
            // -----------------------------------------------------------------
            ACK_SERVANT: begin
                // master must release SDA (tri-state) to let slave drive ACK
                sda_d = 1'b1; // release (for SCCB it's high-impedance semantics)
                if (scl_hi) begin
                    // capture ack: SDA low => ACK (value 1), SDA high => NACK (value 0)
                    ack_tick_d = 1'b1;
                    ack_val_d  = ~sda_q; // sda_q sampled value: 0->ACK(1), 1->NACK(0)

                    // Decide next action: stop, repeated-start read, or continue write
                    if (stop) begin
                        state_d = STOP_1;
                    end else if (start_req_q && read_flag_q) begin
                        // repeated start and R/W=1 (read)
                        start_req_d = 1'b0; // consume the repeated-start request
                        idx_d = 4'd7;
                        state_d = READ;
                    end else begin
                        // continue write path: prepare for next byte (user supplies wr_data)
                        state_d = RENEW_DATA;
                    end
                end
            end

            // -----------------------------------------------------------------
            // RENEW_DATA: prepare for next transmitted byte (write continuation)
            // master expects wr_data to be updated externally before entering PACKET
            // -----------------------------------------------------------------
            RENEW_DATA: begin
                idx_d = 4'd7;
                // When user asserts start_req (repeated start), go to STARTING
                if (start_req_q) begin
                    state_d = STARTING;
                end else begin
                    // load the next byte to transmit from wr_data input
                    wr_data_d = wr_data; // user must present next byte on wr_data
                    state_d = PACKET;
                end
            end

            // -----------------------------------------------------------------
            // READ: sample 8 bits from slave (MSB first) on SCL high middle
            // -----------------------------------------------------------------
            READ: begin
                // release SDA to allow slave to drive data
                sda_d = 1'b1;
                if (scl_hi) begin
                    rd_data_d[idx_q] = sda; // read current bus value (wire)
                    if (idx_q == 0) begin
                        idx_d = 0;
                        state_d = ACK_MASTER;
                    end else begin
                        idx_d = idx_q - 1;
                    end
                end
            end

            // -----------------------------------------------------------------
            // ACK_MASTER: master drives ACK/NACK back to slave during SCL low
            // - SCCB default: don't ACK (drive NACK by leaving SDA=1)
            // - I2C mode: drive ACK low for normal reads; NACK last byte if stop
            // -----------------------------------------------------------------
            ACK_MASTER: begin
                if (scl_lo) begin
                    if (I2C_MODE) begin
                        // for I2C: ACK unless stop is requested (NACK last byte)
                        if (stop)
                            sda_d = 1'b1; // NACK (drive high or release)
                        else
                            sda_d = 1'b0; // ACK (drive low)
                    end else begin
                        // SCCB behavior: don't ACK (NACK)
                        sda_d = 1'b1;
                    end

                    // only set rd_tick when SDA drive choice is accepted (we use sda_q for stability)
                    // produce a registered 1-cycle pulse (rd_tick_d) and store rd_data
                    rd_tick_d = 1'b1;
                    idx_d = 4'd7;

                    if (stop) begin
                        state_d = STOP_1;
                    end else if (start) begin
                        // user asked for repeated start after read
                        start_req_d = 1'b1;
                        state_d = STARTING;
                    end else begin
                        state_d = READ; // read more bytes
                    end
                end
            end

            // -----------------------------------------------------------------
            // STOP sequence: create SDA rising while SCL is high
            // STOP_1: set SDA low while SCL low, then wait for SCL high and raise SDA
            // -----------------------------------------------------------------
            STOP_1: begin
                if (scl_lo) begin
                    sda_d = 1'b0;
                    state_d = STOP_2;
                end
            end

            STOP_2: begin
                if (scl_hi) begin
                    sda_d = 1'b1;
                    state_d = IDLE;
                end
            end

            default: state_d = IDLE;
        endcase
    end

    // -------------------------
    // Tri-state / drive logic for bus lines
    // - SCL: push-pull for SCCB (default), open-drain when I2C_MODE=1
    // - SDA: tri-stated (Z) when released (read / ack-servant), otherwise driven
    // Note: when open-drain mode, driving '1' -> Z (release), '0' -> drive low
    // -------------------------
    // SCL behavior:
    //  - SCCB (I2C_MODE==0): push-pull 0/1 (matches previous default)
    //  - I2C  (I2C_MODE==1): open-drain: drive 0 or 'z' when high
    generate
        if (I2C_MODE) begin : i2c_drive
            assign scl = scl_q ? 1'bz : 1'b0;
        end else begin : sccb_drive
            assign scl = scl_q ? 1'b1 : 1'b0;
        end
    endgenerate

    // SDA behavior: release (Z) during READ or ACK_SERVANT so slave can drive.
    wire sda_released = (state_q == READ || state_q == ACK_SERVANT);

    generate
        if (I2C_MODE) begin : i2c_sda
            // open-drain: when sda_q==1 => release (Z); when 0 => drive low
            assign sda = sda_released ? 1'bz : (sda_q ? 1'bz : 1'b0);
        end else begin : sccb_sda
            // SCCB: push-pull (1 or 0) except when releasing
            assign sda = sda_released ? 1'bz : (sda_q ? 1'b1 : 1'b0);
        end
    endgenerate

    // expose rd_data and state for debug
    assign rd_data = rd_data_q;
    assign state   = state_q[3:0];

endmodule
