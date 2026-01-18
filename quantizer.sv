module quantize_int32_to_int8 #(
    parameter int SHIFT = 8   // divide by 2^SHIFT
)(
    input  logic signed [31:0] in,
    input  logic               in_valid,

    output logic signed [7:0]  out,
    output logic               out_valid
);

    logic signed [31:0] scaled;

    always_comb begin
        if (in_valid) begin
            // 1) scaling
            scaled = in >>> SHIFT;

            // 2) saturation
            if (scaled > 127)
                out = 8'sd127;
            else if (scaled < -128)
                out = -8'sd128;
            else
                out = scaled[7:0];

            out_valid = 1'b1;
        end else begin
            out = '0;
            out_valid = 1'b0;
            scaled = '0;
        end
    end

endmodule
