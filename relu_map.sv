// --------------------------------------------------
// ReLU Feature Map Wrapper
// Applies ReLU to a whole feature map
// --------------------------------------------------
module relu_map #(
    parameter CH = 16,
    parameter IMG_SIZE = 4,
    parameter DATA_W = 32
)(
    input  logic clk,
    input  logic rst,
    input  logic in_valid,

    input  logic signed [DATA_W-1:0] in_map  [CH][IMG_SIZE][IMG_SIZE],
    output logic signed [DATA_W-1:0] out_map [CH][IMG_SIZE][IMG_SIZE],

    output logic out_valid,   // pulses per pixel
    output logic done         // pulses when full map done
);

    logic [$clog2(CH)-1:0] ch;
    logic [$clog2(IMG_SIZE)-1:0] row, col;
    logic [$clog2(CH)-1:0] ch_d;
    logic [$clog2(IMG_SIZE)-1:0] row_d, col_d;


    logic signed [DATA_W-1:0] relu_in, relu_out;
    logic relu_out_valid;

    // Single-pixel ReLU
    relu #(.DATA_W(DATA_W)) relu (
        .in(relu_in),
        .in_valid(in_valid),
        .out(relu_out),
        .out_valid(relu_out_valid)
    );

    always_ff @(posedge clk) begin
    if (rst) begin
        ch <= 0; row <= 0; col <= 0;
        ch_d <= 0; row_d <= 0; col_d <= 0;
        out_valid <= 0;
        done <= 0;
    end
    else if (in_valid) begin
        done <= 0;

        // Feed ReLU
        relu_in <= in_map[ch][row][col];

        // Delay address
        ch_d  <= ch;
        row_d <= row;
        col_d <= col;

        // Move counters
        if (col == IMG_SIZE-1) begin
            col <= 0;
            if (row == IMG_SIZE-1) begin
                row <= 0;
                if (ch == CH-1)
                    ch <= 0;
                else
                    ch <= ch + 1;
            end
            else row <= row + 1;
        end
        else col <= col + 1;

        // Write result from PREVIOUS pixel
        if (relu_out_valid) begin
            out_map[ch_d][row_d][col_d] <= relu_out;
            out_valid <= 1'b1;

            if (ch_d == CH-1 && row_d == IMG_SIZE-1 && col_d == IMG_SIZE-1)
                done <= 1'b1;
        end
        else out_valid <= 0;
    end
    else out_valid <= 0;
end


endmodule
