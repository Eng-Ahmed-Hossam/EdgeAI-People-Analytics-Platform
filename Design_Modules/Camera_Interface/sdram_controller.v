`timescale 1ns / 1ps

module sdram_controller(
    // fpga to controller
    input  wire        clk, rst_n,
    input  wire        rw,           // 1:read , 0:write
    input  wire        rw_en,        // must assert before read/write
    input  wire [14:0] f_addr,       // 14:2=row , 1:0=bank
    input  wire [15:0] f2s_data,     // fpga -> sdram write data
    output wire [15:0] s2f_data,     // sdram -> fpga read data
    output wire        s2f_data_valid,
    output reg         f2s_data_valid,
    output reg         ready,

    // SDRAM pins
    output wire        s_clk,
    output wire        s_cke,
    output wire        s_cs_n, s_ras_n, s_cas_n, s_we_n,
    output wire [12:0] s_addr,
    output wire [1:0]  s_ba,
    output wire        LDQM, HDQM,
    inout  [15:0]      s_dq
);

    /* -------------------- Generate SDRAM Clock -------------------- */
    ODDR2 #(
        .DDR_ALIGNMENT("NONE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) oddr2_primitive (
        .D0(1'b0),
        .D1(1'b1),
        .C0(clk),
        .C1(~clk),
        .CE(1'b1),
        .R(1'b0),
        .S(1'b0),
        .Q(s_clk)
    );

    /* -------------------- FSM States -------------------- */
    localparam [3:0]
        start           = 0,
        precharge_init  = 1,
        refresh_1       = 2,
        refresh_2       = 3,
        load_mode_reg   = 4,
        idle            = 5,
        read            = 6,
        read_data       = 7,
        write           = 8,
        write_burst     = 9,
        refresh         = 10,
        delay           = 11;

    /* -------------------- SDRAM Timing (165 MHz) -------------------- */
    localparam integer
        t_RP  = 2,
        t_RC  = 7,
        t_MRD = 2,
        t_RCD = 2,
        t_WR  = 2,
        t_CL  = 3;

    /* -------------------- SDRAM Commands -------------------- */
    localparam [3:0]
        cmd_precharge = 4'b0010,
        cmd_NOP       = 4'b1111,
        cmd_activate  = 4'b0011,
        cmd_write     = 4'b0100,
        cmd_read      = 4'b0101,
        cmd_setmode   = 4'b0000,
        cmd_refresh   = 4'b0001;

    /* -------------------- Registers -------------------- */
    reg [3:0]  state_q, state_d;
    reg [3:0]  nxt_q,   nxt_d;
    reg [3:0]  cmd_q,   cmd_d;

    reg [15:0] delay_ctr_q, delay_ctr_d;
    reg [10:0] refresh_ctr_q, refresh_ctr_d;
    reg        refresh_flag_q, refresh_flag_d;

    reg [9:0]  burst_index_q, burst_index_d;

    reg        tri_q, tri_d;

    reg [14:0] f_addr_q, f_addr_d;
    reg [15:0] f2s_data_q, f2s_data_d;
    reg [15:0] s2f_data_q, s2f_data_d;
    reg        s2f_data_valid_q, s2f_data_valid_d;

    reg        rw_q, rw_d;
    reg        rw_en_q, rw_en_d;

    reg [12:0] s_addr_q, s_addr_d;
    reg [1:0]  s_ba_q, s_ba_d;
    reg [15:0] s_dq_q, s_dq_d;

    /* -------------------- Sequential Logic -------------------- */
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state_q           <= start;
            nxt_q             <= start;
            cmd_q             <= cmd_NOP;
            delay_ctr_q       <= 0;
            refresh_ctr_q     <= 0;
            refresh_flag_q    <= 0;
            burst_index_q     <= 0;

            tri_q             <= 0;
            s_addr_q          <= 0;
            s_ba_q            <= 0;
            s_dq_q            <= 0;

            f_addr_q          <= 0;
            f2s_data_q        <= 0;
            s2f_data_q        <= 0;
            s2f_data_valid_q  <= 0;

            rw_q              <= 0;
            rw_en_q           <= 0;

        end else begin
            state_q           <= state_d;
            nxt_q             <= nxt_d;
            cmd_q             <= cmd_d;
            delay_ctr_q       <= delay_ctr_d;
            refresh_ctr_q     <= refresh_ctr_d;
            refresh_flag_q    <= refresh_flag_d;
            burst_index_q     <= burst_index_d;

            tri_q             <= tri_d;
            s_addr_q          <= s_addr_d;
            s_ba_q            <= s_ba_d;
            s_dq_q            <= s_dq_d;

            f_addr_q          <= f_addr_d;
            f2s_data_q        <= f2s_data_d;
            s2f_data_q        <= s2f_data_d;
            s2f_data_valid_q  <= s2f_data_valid_d;

            rw_q              <= rw_d;
            rw_en_q           <= rw_en_d;
        end
    end

    /* -------------------- Combinational Logic -------------------- */
    always @(*) begin
        state_d          = state_q;
        nxt_d            = nxt_q;
        cmd_d            = cmd_NOP;

        delay_ctr_d      = delay_ctr_q;
        refresh_ctr_d    = refresh_ctr_q + 1'b1;
        refresh_flag_d   = refresh_flag_q;

        tri_d            = 0;
        ready            = 0;

        s_addr_d         = s_addr_q;
        s_ba_d           = s_ba_q;
        s_dq_d           = s_dq_q;

        f_addr_d         = f_addr_q;
        f2s_data_d       = f2s_data_q;
        s2f_data_d       = s2f_data_q;
        s2f_data_valid_d = 1'b0;

        rw_d             = rw_q;
        rw_en_d          = rw_en_q;

        burst_index_d    = burst_index_q;

        /* --- Refresh every 7.8 µs (1283 clocks @ 165MHz) --- */
        if(refresh_ctr_q == 1283) begin
            refresh_ctr_d  = 0;
            refresh_flag_d = 1;
        end

        /* ===================== FSM ===================== */

        case(state_q)

            /* -------- Delay State -------- */
            delay: begin
                delay_ctr_d = delay_ctr_q - 1'b1;
                if(delay_ctr_d == 0)
                    state_d = nxt_q;

                if(nxt_q == write)
                    tri_d = 1;
            end

            /* -------- Initialization Sequence -------- */
            start: begin
                state_d      = delay;
                nxt_d        = precharge_init;
                delay_ctr_d  = 16'd33000;     // 200us
                s_addr_d     = 0;
                s_ba_d       = 0;
            end

            precharge_init: begin
                state_d      = delay;
                nxt_d        = refresh_1;
                delay_ctr_d  = t_RP - 1;
                cmd_d        = cmd_precharge;
                s_addr_d[10] = 1'b1;
            end

            refresh_1: begin
                state_d      = delay;
                nxt_d        = refresh_2;
                delay_ctr_d  = t_RC - 1;
                cmd_d        = cmd_refresh;
            end

            refresh_2: begin
                state_d      = delay;
                nxt_d        = load_mode_reg;
                delay_ctr_d  = t_RC - 1;
                cmd_d        = cmd_refresh;
            end

            load_mode_reg: begin
                state_d      = delay;
                nxt_d        = idle;
                delay_ctr_d  = t_MRD - 1;
                cmd_d        = cmd_setmode;
                s_addr_d     = 13'b0000_00_011_0_111;
                s_ba_d       = 0;
            end

            /* -------- Idle -------- */
            idle: begin
                ready = !rw_en_q;

                /* immediate refresh */
                if(refresh_flag_q) begin
                    state_d      = delay;
                    nxt_d        = refresh;
                    delay_ctr_d  = t_RP - 1;
                    cmd_d        = cmd_precharge;
                    s_addr_d[10] = 1;
                    refresh_flag_d = 0;
                end

                /* request r/w */
                else if(rw_en_q) begin
                    state_d      = delay;
                    nxt_d        = rw_q ? read : write;
                    delay_ctr_d  = t_RCD - 1;
                    burst_index_d = 0;

                    {s_addr_d, s_ba_d} = f_addr_q;
                    cmd_d = cmd_activate;
                    rw_en_d = 0;
                end

                /* external request received now */
                else if(rw_en) begin
                    f_addr_d    = f_addr;
                    rw_d        = rw;
                    rw_en_d     = 1;
                end
            end

            /* -------- Auto Refresh -------- */
            refresh: begin
                state_d     = delay;
                nxt_d       = idle;
                delay_ctr_d = t_RC - 1;
                cmd_d       = cmd_refresh;
            end

            /* -------- READ Command -------- */
            read: begin
                state_d      = delay;
                delay_ctr_d  = t_CL - 1;
                cmd_d        = cmd_read;

                s_addr_d     = 0;
                s_ba_d       = f_addr_q[1:0];
                s_addr_d[10] = 0;

                nxt_d        = read_data;
            end

            read_data: begin
                s2f_data_d       = s_dq;
                s2f_data_valid_d = 1'b1;

                burst_index_d    = burst_index_q + 1;

                if(burst_index_q == 511) begin
                    state_d      = delay;
                    nxt_d        = idle;
                    delay_ctr_d  = t_RP - 1;
                    cmd_d        = cmd_precharge;
                end
            end

            /* -------- WRITE Command -------- */
            write: begin
                f2s_data_d     = f2s_data;
                f2s_data_valid = 1'b1;

                s_addr_d       = 0;
                s_ba_d         = f_addr_q[1:0];
                s_addr_d[10]   = 0;

                tri_d          = 1;
                cmd_d          = cmd_write;

                burst_index_d  = 1;
                state_d        = write_burst;
            end

            write_burst: begin
                f2s_data_d     = f2s_data;
                f2s_data_valid = 1'b1;
                tri_d          = 1;

                burst_index_d  = burst_index_q + 1;

                if(burst_index_q == 511) begin
                    tri_d        = 0;
                    f2s_data_valid = 0;

                    state_d      = delay;
                    nxt_d        = idle;
                    delay_ctr_d  = t_RP + t_WR - 1;
                    cmd_d        = cmd_precharge;
                end
            end

        endcase
    end

    /* -------------------- Output Assignments -------------------- */
    assign s_cs_n  = cmd_q[3];
    assign s_ras_n = cmd_q[2];
    assign s_cas_n = cmd_q[1];
    assign s_we_n  = cmd_q[0];

    assign s_cke   = 1'b1;

    assign LDQM    = 1'b0;
    assign HDQM    = 1'b0;

    assign s_addr  = s_addr_q;
    assign s_ba    = s_ba_q;

    assign s_dq    = tri_q ? f2s_data_q : 16'hzzzz;

    assign s2f_data        = s2f_data_q;
    assign s2f_data_valid  = s2f_data_valid_q;

endmodule
