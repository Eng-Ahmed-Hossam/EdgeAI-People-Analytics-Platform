module timer_20ms (
    input  wire clk,
    input  wire rst,
    input  wire start_timer,
    output reg  timer_done
);
    // 2,000,000 cycles for 20ms at 100MHz , waiting for measurement
    reg [21:0] count; 

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            count <= 0;
            timer_done <= 0;
        end else if (start_timer) begin
            if (count == 2000000) begin
                timer_done <= 1;
                count <= count;
            end else begin
                count <= count + 1;
                timer_done <= 0;
            end
        end else begin
            count <= 0;
            timer_done <= 0;
        end
    end
endmodule