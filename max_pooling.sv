module max_pooling #(
    parameter DATA_WIDTH = 8,
    parameter IMG_SIZE = 4,
    parameter POOL_SIZE = 2,
    parameter STRIDE = 2,
    parameter CH_IN = 3,
    parameter PADDING = 1
)(
    input logic clk,
    input logic rst,
    input logic [DATA_WIDTH-1:0] img[CH_IN-1:0][IMG_SIZE-1:0][IMG_SIZE-1:0],
    output logic [DATA_WIDTH-1:0] out[CH_IN-1:0][((IMG_SIZE + 2*PADDING-POOL_SIZE)/STRIDE + 1)-1:0][((IMG_SIZE + 2*PADDING-POOL_SIZE)/STRIDE + 1)-1:0]
);
logic [DATA_WIDTH-1:0] padded_img[CH_IN-1:0][IMG_SIZE + 2*PADDING - 1][IMG_SIZE + 2*PADDING - 1];

//padding 
always_comb begin
    for(int c = 0; c < CH_IN; c++)begin
        for(int i = 0; i < IMG_SIZE + 2*PADDING; i++)begin
            for(int j = 0; j < IMG_SIZE + 2*PADDING; j++)begin
                padded_img[c][i][j] = 0;
            end
        end
    end
    
    for(int c = 0; c < CH_IN; c++)begin
        for(int i = 0; i < IMG_SIZE ; i++)begin
            for(int j = 0; j < IMG_SIZE ; j++)begin
                padded_img[c][i+PADDING][j+PADDING] = img[c][i][j];
            end
        end
    end
end

//max pooling operation
always_ff @(posedge clk)begin
if(rst)begin
    for(int c = 0; c < CH_IN ;c++)begin
        for(int i = 0; i < IMG_SIZE+2*PADDING-POOL_SIZE; i = i+STRIDE)begin
            for(int j = 0; j < IMG_SIZE+2*PADDING-POOL_SIZE; j = j+STRIDE)begin
                out[c][i/STRIDE][j/STRIDE] <= 0;  
            end
        end
    end
end
else begin
    for (int c = 0; c < CH_IN; c++) begin
            for (int i = 0; i <= IMG_SIZE + 2*PADDING - POOL_S
            IZE; i = i + STRIDE) begin
                for (int j = 0; j <= IMG_SIZE + 2*PADDING - POOL_SIZE; j = j + STRIDE) begin
                    logic [DATA_WIDTH-1:0] max_val;
                    max_val = padded_img[c][i][j];
                    for (int m = 0; m < POOL_SIZE; m = m + 1)
                        for (int n = 0; n < POOL_SIZE; n = n + 1)
                            if (padded_img[c][i+m][j+n] > max_val)
                                max_val = padded_img[c][i+m][j+n];
                    out[c][i/STRIDE][j/STRIDE] <= max_val;
                end
            end
        end
end
end
endmodule