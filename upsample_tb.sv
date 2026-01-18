`timescale 1ns/1ps
module tb_upsample;

    parameter CH = 3;
    parameter IN_SIZE = 3;
    parameter FACTOR = 2;

    logic clk;
    logic rst;
    logic in_valid;
    logic signed [7:0] in_map [CH][IN_SIZE][IN_SIZE];
    logic signed [7:0] out_map [CH][IN_SIZE*FACTOR][IN_SIZE*FACTOR];
    logic out_valid;
    logic done;

    // Instantiate the module
    upsample #(.CH(CH), .IN_SIZE(IN_SIZE), .FACTOR(FACTOR)) uut (
        .clk(clk),
        .rst(rst),
        .in_valid(in_valid),
        .in_map(in_map),
        .out_map(out_map),
        .out_valid(out_valid),
        .done(done)
    );

    // Clock generation
    always #5 clk = ~clk;

    initial begin
        clk = 0;
        rst = 1;
        in_valid = 0;
        #10;
        rst = 0;

        // Initialize 3x3, 3-channel input
        // Channel 0
        in_map[0][0] = '{1, 2, 3};
        in_map[0][1] = '{4, 5, 6};
        in_map[0][2] = '{7, 8, 9};

        // Channel 1
        in_map[1][0] = '{10, 11, 12};
        in_map[1][1] = '{13, 14, 15};
        in_map[1][2] = '{16, 17, 18};

        // Channel 2
        in_map[2][0] = '{20, 21, 22};
        in_map[2][1] = '{23, 24, 25};
        in_map[2][2] = '{26, 27, 28};

        // Apply input
        @(posedge clk);
        in_valid = 1;
        @(posedge clk);
        in_valid = 0;

        // Wait a bit
        #10;

        // Display output
        for (int c = 0; c < CH; c++) begin
            $display("Channel %0d upsampled output:", c);
            for (int i = 0; i < IN_SIZE*FACTOR; i++) begin
                for (int j = 0; j < IN_SIZE*FACTOR; j++)
                    $write("%0d ", out_map[c][i][j]);
                $write("\n");
            end
            $display("\n");
        end

        #10 $finish;
    end
endmodule
