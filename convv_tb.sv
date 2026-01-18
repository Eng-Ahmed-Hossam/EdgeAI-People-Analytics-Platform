`timescale 1ns/1ps

module tb_conv;

    // ================= Parameters =================
    parameter CH_IN    = 1;
    parameter CH_OUT   = 1;
    parameter IMG_SIZE = 4;
    parameter K        = 3;
    parameter PADDING  = 1;
    parameter STRIDE   = 1;
    parameter PIXEL_SIZE_IN  = 8;
    parameter PIXEL_SIZE_OUT = 32;

    localparam OUT_SIZE = (IMG_SIZE + 2*PADDING - K)/STRIDE + 1;

    // ================= Signals =================
    logic clk;
    logic rst;
    logic in_valid;

    logic signed [PIXEL_SIZE_IN-1:0]  image [CH_IN][IMG_SIZE][IMG_SIZE];
    logic signed [PIXEL_SIZE_IN-1:0]  filter[CH_OUT][CH_IN][K][K];
    logic signed [PIXEL_SIZE_OUT-1:0] bias  [CH_OUT];

    logic signed [PIXEL_SIZE_OUT-1:0] op_img[CH_OUT][OUT_SIZE][OUT_SIZE];
    logic out_valid;
    logic done;

    // ================= Clock =================
    initial clk = 0;
    always #5 clk = ~clk;

    // ================= DUT =================
    conv #(
        .CH_IN(CH_IN),
        .CH_OUT(CH_OUT),
        .IMG_SIZE(IMG_SIZE),
        .K(K),
        .PADDING(PADDING),
        .STRIDE(STRIDE),
        .PIXEL_SIZE_IN(PIXEL_SIZE_IN),
        .PIXEL_SIZE_OUT(PIXEL_SIZE_OUT)
    ) dut (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .image(image),
        .filter(filter),
        .bias(bias),
        .op_img(op_img),
        .out_valid(out_valid),
        .done(done)
    );

    // ================= Test Data =================
    initial begin
        // -------- Input Image (1 channel) --------
        image[0][0] = '{1, 2, 3, 4};
        image[0][1] = '{5, 6, 7, 8};
        image[0][2] = '{9,10,11,12};
        image[0][3] = '{13,14,15,16};

        // -------- Filter = all ones --------
        for (int ci = 0; ci < CH_IN; ci++)
            for (int m = 0; m < K; m++)
                for (int n = 0; n < K; n++)
                    filter[0][ci][m][n] = 1;

        // -------- Bias = 0 --------
        bias[0] = 0;
    end

    // ================= Stimulus =================
    initial begin
        rst = 1;
        in_valid = 0;
        #20;

        rst = 0;
        #10;

        in_valid = 1;      // start convolution
        wait(done);
        #10;

        in_valid = 0;
    end

    // ================= Output Monitor =================
    always @(posedge done) begin
        $display("===== Convolution Output =====");
        for (int i = 0; i < OUT_SIZE; i++) begin
            for (int j = 0; j < OUT_SIZE; j++)
                $write("%0d\t", op_img[0][i][j]);
            $write("\n");
        end
        $finish;
    end

endmodule