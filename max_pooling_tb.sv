`timescale 1ns/1ps
module tb_max_pooling;

    parameter DATA_WIDTH = 8;
    parameter IMG_SIZE = 4;
    parameter POOL_SIZE = 2;
    parameter STRIDE = 2;
    parameter CH_IN = 3;
    parameter PADDING = 1;

    logic clk;
    logic rst;
    logic [DATA_WIDTH-1:0] img[CH_IN-1:0][IMG_SIZE-1:0][IMG_SIZE-1:0];
    logic [DATA_WIDTH-1:0] out[CH_IN-1:0][((IMG_SIZE + 2*PADDING-POOL_SIZE)/STRIDE + 1)-1:0]
                           [((IMG_SIZE + 2*PADDING-POOL_SIZE)/STRIDE + 1)-1:0];

    // instantiate max_pooling
    max_pooling #(
        .DATA_WIDTH(DATA_WIDTH),
        .IMG_SIZE(IMG_SIZE),
        .POOL_SIZE(POOL_SIZE),
        .STRIDE(STRIDE),
        .CH_IN(CH_IN),
        .PADDING(PADDING)
    ) uut (
        .clk(clk),
        .rst(rst),
        .img(img),
        .out(out)
    );

    // Clock generation
    initial clk = 0;
    always #5 clk = ~clk; // 10ns period

    // Initialize inputs
    initial begin
        rst = 1;
        #10;
        rst = 0;

        // Example image data (3 channels, 4x4)
        // Channel 0
        img[0] = '{'{1,2,3,4},
                   '{5,6,7,8},
                   '{9,10,11,12},
                   '{13,14,15,16}};
        // Channel 1
        img[1] = '{'{16,15,14,13},
                   '{12,11,10,9},
                   '{8,7,6,5},
                   '{4,3,2,1}};
        // Channel 2
        img[2] = '{'{1,3,5,7},
                   '{2,4,6,8},
                   '{9,11,13,15},
                   '{10,12,14,16}};

        #20; // wait for pooling operation
        $display("=== Max Pooling Output ===");
        for (int c = 0; c < CH_IN; c++) begin
            $display("Channel %0d:", c);
            for (int i = 0; i <= IMG_SIZE + 2*PADDING - POOL_SIZE; i = i + STRIDE) begin
                for (int j = 0; j <= IMG_SIZE + 2*PADDING - POOL_SIZE; j = j + STRIDE) begin
                    $write("%0d ", out[c][i/STRIDE][j/STRIDE]);
                end
                $write("\n");
            end
            $write("\n");
        end

        $stop;
    end

endmodule
