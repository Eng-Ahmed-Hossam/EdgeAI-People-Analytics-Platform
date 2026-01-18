module conv #(
    parameter CH_IN = 3,
    parameter CH_OUT = 16,
    parameter PADDING = 1,
    parameter STRIDE = 1,
    parameter IMG_SIZE = 4,
    parameter K = 3,
    parameter PIXEL_SIZE_IN = 8,
    parameter PIXEL_SIZE_OUT = 32
)(
    input  logic in_valid,
    input  logic clk,
    input  logic rst,

    // Input image
    input  logic signed [PIXEL_SIZE_IN-1:0] image[CH_IN][IMG_SIZE][IMG_SIZE],

    // Filters and bias passed from top module
    input  logic signed [PIXEL_SIZE_IN-1:0] filter[CH_OUT][CH_IN][K][K],
    input  logic signed [PIXEL_SIZE_OUT-1:0] bias[CH_OUT],

    // Output
    output logic signed [PIXEL_SIZE_OUT-1:0] op_img[CH_OUT][(IMG_SIZE + 2*PADDING - K)/STRIDE + 1][(IMG_SIZE + 2*PADDING - K)/STRIDE + 1],
    output logic out_valid,
    output logic done
);

    localparam PADDED = IMG_SIZE + 2*PADDING;
    localparam OUT_SIZE = (IMG_SIZE + 2*PADDING - K)/STRIDE + 1;

    // padded image
    logic signed [PIXEL_SIZE_IN-1:0] padded_img[CH_IN][PADDED][PADDED];

    // intermediate sum
    logic signed [PIXEL_SIZE_OUT:0] sum_comb[CH_OUT];
    logic signed [PIXEL_SIZE_OUT:0] sum_bias[CH_OUT];

    logic [$clog2(OUT_SIZE)-1:0] row, col;

    // -------------------------------
    // Convolution math
    // -------------------------------
    always_comb begin
        for (int co = 0; co < CH_OUT; co++) begin
            sum_comb[co] = '0;
            for (int ci = 0; ci < CH_IN; ci++)
                for (int m = 0; m < K; m++)
                    for (int n = 0; n < K; n++)
                        sum_comb[co] += 
                            $signed(padded_img[ci][row*STRIDE+m][col*STRIDE+n]) *
                            $signed(filter[co][ci][m][n]);

            sum_bias[co] = sum_comb[co] + bias[co];
        end
    end

    // -------------------------------
    // Control + memory update
    // -------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            row <= 0;
            col <= 0;
            out_valid <= 0;
            done <= 0;

            // clear padded image
            for (int c = 0; c < CH_IN; c++)
                for (int i = 0; i < PADDED; i++)
                    for (int j = 0; j < PADDED; j++)
                        padded_img[c][i][j] <= '0;

            // copy image into padded memory
            for (int c = 0; c < CH_IN; c++)
                for (int i = 0; i < IMG_SIZE; i++)
                    for (int j = 0; j < IMG_SIZE; j++)
                        padded_img[c][i+PADDING][j+PADDING] <= image[c][i][j];

            // clear output
            for (int co = 0; co < CH_OUT; co++)
                for (int i = 0; i < OUT_SIZE; i++)
                    for (int j = 0; j < OUT_SIZE; j++)
                        op_img[co][i][j] <= '0;

        end else begin
            out_valid <= 0;
            done <= 0;

            if (in_valid) begin
                out_valid <= 1'b1;

                // write one output pixel for all channels
                for (int co = 0; co < CH_OUT; co++)
                    op_img[co][row][col] <= sum_bias[co][PIXEL_SIZE_OUT-1:0];

                // scanning logic
                if (col == OUT_SIZE-1) begin
                    col <= 0;
                    if (row == OUT_SIZE-1) begin
                        row <= 0;
                        done <= 1'b1;
                    end else row <= row + 1;
                end else col <= col + 1;
            end
        end
    end

endmodule
