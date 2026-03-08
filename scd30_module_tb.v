`timescale 1ns/1ps

module scd30_co2_driver_tb();
    // إشارات الإدخال للدرايفر (Reg)
    reg clk;
    reg reset;
    reg start;
    reg i2c_ready;
    reg [7:0] i2c_data_out;

    // إشارات الإخراج من الدرايفر (Wire)
    wire [15:0] co2_data;
    wire done;
    wire [6:0] i2c_addr;
    wire [7:0] i2c_data_in;
    wire i2c_enable;
    wire i2c_rw;

    // استدعاء موديول الدرايفر
    scd30_co2_driver uut (
        .clk(clk),
        .reset(reset),
        .start(start),
        .co2_data(co2_data),
        .done(done),
        .i2c_addr(i2c_addr),
        .i2c_data_in(i2c_data_in),
        .i2c_enable(i2c_enable),
        .i2c_rw(i2c_rw),
        .i2c_ready(i2c_ready),
        .i2c_data_out(i2c_data_out)
    );

    // توليد نبضات الساعة (100MHz)
    always #5 clk = ~clk;

    initial begin
        // --- 1. البداية (Initialization) ---
        clk = 0;
        reset = 1;
        start = 0;
        i2c_ready = 1; // الماستر في البداية بيبقى جاهز
        i2c_data_out = 8'h00;

        #100 reset = 0;
        #50;

        // --- 2. إعطاء أمر البدء (Start Pulse) ---
        @(posedge clk);
        start = 1;
        @(posedge clk);
        start = 0;
        $display("Time: %t | Start signal sent", $time);

        // --- 3. محاكاة قراءة البايت الأول (MSB) ---
        // ننتظر الدرايفر يرفع الـ enable
        wait(i2c_enable == 1);
        #20;
        i2c_ready = 0; // محاكاة إن الماستر بدأ يشتغل فعلاً
        #100;
        i2c_data_out = 8'h12; // القيمة اللي المفروض الحساس يبعتها (MSB)
        i2c_ready = 1;        // محاكاة إن الماستر خلص قراءة أول بايت
        $display("Time: %t | MSB Byte (0x12) provided to driver", $time);

        // --- 4. محاكاة قراءة البايت الثاني (LSB) ---
        // ننتظر الدرايفر يطلب البايت التاني
        wait(i2c_enable == 1);
        #20;
        i2c_ready = 0;
        #100;
        i2c_data_out = 8'h34; // القيمة التانية (LSB)
        i2c_ready = 1;
        $display("Time: %t | LSB Byte (0x34) provided to driver", $time);

        // --- 5. التأكد من النتيجة النهائية ---
        wait(done == 1);
        $display("---------------------------------------");
        $display("Final CO2 Data Captured: 0x%h", co2_data);
        if (co2_data == 16'h1234)
            $display("TEST RESULT: SUCCESS");
        else
            $display("TEST RESULT: FAILED");
        $display("---------------------------------------");

        #100 $stop;
    end

endmodule