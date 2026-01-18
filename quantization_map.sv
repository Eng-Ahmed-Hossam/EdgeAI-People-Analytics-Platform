module quant_map #(
    parameter CH = 16,
    parameter IMG_SIZE = 4,
    parameter SHIFT = 8
)(
    input  logic clk,
    input  logic rst,
    input  logic in_valid,

    input  logic signed [31:0] in_map  [CH][IMG_SIZE][IMG_SIZE],
    output logic signed [7:0]  out_map [CH][IMG_SIZE][IMG_SIZE],

    output logic out_valid,
    output logic done
);

    logic [$clog2(CH)-1:0] ch;
    logic [$clog2(IMG_SIZE)-1:0] row, col;

    logic signed [31:0] q_in;
    logic signed [7:0]  q_out;
    logic q_out_valid;

    logic [$clog2(CH)-1:0] ch_d;
    logic [$clog2(IMG_SIZE)-1:0] row_d, col_d;

    logic last_in, last_in_d;

    // -------------------------
    // Quantizer
    // -------------------------
    quantize_int32_to_int8 #(.SHIFT(SHIFT)) Q (
        .in(q_in),
        .in_valid(in_valid),
        .out(q_out),
        .out_valid(q_out_valid)
    );

    // detect last input pixel
    always_comb begin
        last_in = (ch == CH-1) &&
                  (row == IMG_SIZE-1) &&
                  (col == IMG_SIZE-1);
    end

    // delay last flag
    always_ff @(posedge clk) begin
        if (rst)
            last_in_d <= 1'b0;
        else if (in_valid)
            last_in_d <= last_in;
    end

    // -------------------------
    // Control logic
    // -------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            ch <= 0; row <= 0; col <= 0;
            ch_d <= 0; row_d <= 0; col_d <= 0;
            out_valid <= 0;
            done <= 0;
        end
        else if (in_valid) begin
            // feed quantizer
            q_in <= in_map[ch][row][col];

            // delay indices
            ch_d  <= ch;
            row_d <= row;
            col_d <= col;

            out_valid <= 0;
            done <= 0;

            if (q_out_valid) begin
                out_map[ch_d][row_d][col_d] <= q_out;
                out_valid <= 1'b1;

                // assert done ONLY when last output is written
                if (last_in_d)
                    done <= 1'b1;

                // move counters ONLY while inputs still exist
                if (!last_in) begin
                    if (col == IMG_SIZE-1) begin
                        col <= 0;
                        if (row == IMG_SIZE-1) begin
                            row <= 0;
                            ch <= ch + 1;
                        end else row <= row + 1;
                    end else col <= col + 1;
                end
            end
        end
        else out_valid <= 0;
    end

endmodule
