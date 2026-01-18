module max_pooling #(
    parameter IMG_SIZE = 4,
    parameter CH_IN    = 3,
    parameter CH_OUT   = 3, // CH_IN = CH_OUT in pooling
    parameter K        = 2, // pooling window size
    parameter STRIDE   = 2
)(
    input  logic clk,
    input  logic rst,
    input  logic in_valid,  // ✅ start / enable pooling
    input  logic signed [7:0] x[CH_IN][IMG_SIZE][IMG_SIZE],
    output logic signed [7:0] pooled_output[CH_OUT][(IMG_SIZE-K)/STRIDE+1][(IMG_SIZE-K)/STRIDE+1],
    output logic out_valid,  // pulses per output pixel
    output logic done        // pulses when full pooled map is done
);

    localparam OUT_SIZE = (IMG_SIZE - K)/STRIDE + 1;
    logic [$clog2(OUT_SIZE)-1:0] row, col;

    // -------------------------------
    // Control + pooling computation
    // -------------------------------
    always_ff @(posedge clk) begin
        if (rst) begin
            row <= 0;
            col <= 0;
            out_valid <= 0;
            done <= 0;
            // clear output
            for (int ch = 0; ch < CH_OUT; ch++)
                for (int r = 0; r < OUT_SIZE; r++)
                    for (int c = 0; c < OUT_SIZE; c++)
                        pooled_output[ch][r][c] <= '0;
        end
        else begin
            out_valid <= 0;
            done <= 0;

            if (in_valid) begin
                // compute pooling for current row,col
                for (int ch = 0; ch < CH_IN; ch++) begin
                    int signed max_val;
                    max_val = x[ch][row*STRIDE][col*STRIDE]; // first element

                    for (int m = 0; m < K; m++)
                        for (int n = 0; n < K; n++) begin
                            int signed val;
                            val = x[ch][row*STRIDE + m][col*STRIDE + n];
                            if (val > max_val)
                                max_val = val;
                        end

                    pooled_output[ch][row][col] <= max_val;
                end

                out_valid <= 1'b1; // pulse: one output pixel ready

                // -------------------------------
                // scan to next pixel
                // -------------------------------
                if (col == OUT_SIZE-1) begin
                    col <= 0;
                    if (row == OUT_SIZE-1) begin
                        row <= 0;
                        done <= 1'b1; // full pooled map done
                    end else
                        row <= row + 1;
                end else
                    col <= col + 1;
            end
        end
    end

endmodule
