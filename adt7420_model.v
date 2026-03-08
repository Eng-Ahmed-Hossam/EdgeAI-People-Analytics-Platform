module ADT7420_Driver (
    input clk, reset,
    output reg [15:0] temperature, // هنا ستخزن درجة الحرارة كاملة
    output reg data_valid,
    
    // التوصيل مع الـ I2C Master الذي صنعناه
    output reg i2c_enable,
    output reg i2c_rw,
    output reg [7:0] i2c_data_in,
    input [7:0] i2c_data_out,
    input i2c_ready
);

    reg [2:0] state = 0;
    localparam IDLE = 0, ADDR_REG = 1, READ_MSB = 2, READ_LSB = 3, DONE = 4;

    always @(posedge clk or posedge reset) begin
        if(reset) begin
            state <= IDLE;
            i2c_enable <= 0;
            temperature <= 0;
            data_valid <= 0;
        end else begin
            case(state)
                IDLE: begin
                    data_valid <= 0;
                    if(i2c_ready) begin
                        i2c_data_in <= 8'h00; // عنوان سجل الحرارة داخل السينسور
                        i2c_rw <= 0;          // أمر كتابة لاختيار السجل
                        i2c_enable <= 1;
                        state <= ADDR_REG;
                    end
                end
                
                ADDR_REG: begin
                    if(!i2c_ready) i2c_enable <= 0; // انتظر حتى يبدأ الماستر
                    if(i2c_ready && !i2c_enable) begin
                        i2c_rw <= 1;          // الآن نطلب قراءة
                        i2c_enable <= 1;
                        state <= READ_MSB;
                    end
                end

                READ_MSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        temperature[15:8] <= i2c_data_out; // حفظ البايت الأول
                        i2c_enable <= 1;      // طلب البايت الثاني
                        state <= READ_LSB;
                    end
                end

                READ_LSB: begin
                    if(!i2c_ready) i2c_enable <= 0;
                    if(i2c_ready && !i2c_enable) begin
                        temperature[7:0] <= i2c_data_out;  // حفظ البايت الثاني
                        state <= DONE;
                    end
                end

                DONE: begin
                    data_valid <= 1;
                    state <= IDLE; // كرر العملية باستمرار
                end
            endcase
        end
    end
endmodule