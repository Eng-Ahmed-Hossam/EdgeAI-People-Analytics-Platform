// --------------------------------------------------
// ReLU Activation Unit (Parametrized)
// --------------------------------------------------
module relu #(
    parameter integer DATA_W = 32
)(
    input  wire signed [DATA_W-1:0] in,
    input  wire                     in_valid,

    output reg  signed [DATA_W-1:0] out,
    output reg                      out_valid
);

    always @(*) begin
        if (in_valid) begin
            if (in[DATA_W-1] == 1'b1) // negative
                out = 'd0;
            else
                out = in;
            out_valid = 1'b1;  // ✅ fixed
        end else begin
            out = 'd0;
            out_valid = 1'b0;
        end
    end

endmodule
