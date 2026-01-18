module upsample #(
    parameter CH = 3,
    parameter IN_SIZE = 3,
    parameter FACTOR = 2
)(
    input  logic clk,
    input  logic rst,
    input  logic in_valid,
    input  logic signed [7:0] in_map [CH][IN_SIZE][IN_SIZE],
    output logic signed [7:0] out_map [CH][IN_SIZE*FACTOR][IN_SIZE*FACTOR],
    output logic out_valid,
    output logic done
);

    always_ff @(posedge clk) begin
        if (rst) begin
            out_valid <= 0;
            done <= 0;
            // Clear output
            for (int c = 0; c < CH; c++)
                for (int r = 0; r < IN_SIZE*FACTOR; r++)
                    for (int co = 0; co < IN_SIZE*FACTOR; co++)
                        out_map[c][r][co] <= 0;
        end else if (in_valid) begin
            // Upsample each channel in parallel
            for (int c = 0; c < CH; c++)
                for (int r = 0; r < IN_SIZE; r++)
                    for (int co = 0; co < IN_SIZE; co++) begin
                        out_map[c][r*FACTOR][co*FACTOR]       <= in_map[c][r][co];
                        out_map[c][r*FACTOR][co*FACTOR + 1]   <= in_map[c][r][co];
                        out_map[c][r*FACTOR + 1][co*FACTOR]   <= in_map[c][r][co];
                        out_map[c][r*FACTOR + 1][co*FACTOR + 1] <= in_map[c][r][co];
                    end
            out_valid <= 1;
            done <= 1;
        end else begin
            out_valid <= 0;
            done <= 0;
        end
    end
endmodule
