module Temperature_System (
    input clk,          // ساعة الـ FPGA (100MHz)
    input reset,        // زرار الـ Reset
    output [15:0] temp_out, // القيمة النهائية لعرضها (مثلاً على الـ LEDs)
    output data_ready,  // لمبة تنور لما القراءة تخلص
    
    // الأرجل الفعلية اللي هتتوصل بالسينسور على البوردة
    output i2c_scl,
    inout  i2c_sda
);

    // (Internal Wires)
    wire [7:0] w_i2c_data_to_master;
    wire [7:0] w_i2c_data_from_master;
    wire w_i2c_enable;
    wire w_i2c_rw;
    wire w_i2c_ready;

    // 1. استدعاء موديول الـ Driver (المخ)
    ADT7420_Driver driver_inst (
        .clk(clk),
        .reset(reset),
        .temperature(temp_out),
        .data_valid(data_ready),
        .i2c_enable(w_i2c_enable),
        .i2c_rw(w_i2c_rw),
        .i2c_data_in(w_i2c_data_to_master),
        .i2c_data_out(w_i2c_data_from_master),
        .i2c_ready(w_i2c_ready)
    );

    // 2. استدعاء موديول الـ Master (المحرك)
    i2c_master master_inst (
        .clk(clk),
        .reset(reset),
        .addr(7'h4B),          // عنوان السينسور ثابت
        .data_in(w_i2c_data_to_master),
        .enable(w_i2c_enable),
        .rw(w_i2c_rw),
        .data_out(w_i2c_data_from_master),
        .ready(w_i2c_ready),
        .ack_error(),          // ممكن توصليها بـ LED للتحذير
        .i2c_scl(i2c_scl),
        .i2c_sda(i2c_sda)
    );

endmodule