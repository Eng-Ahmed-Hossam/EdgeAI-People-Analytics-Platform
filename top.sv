module top_cnn_24layers (
    input  logic clk,
    input  logic rst,
    input  logic in_valid,
    input  logic signed [7:0] image_in [3][416][416],
    output logic done,
    output logic signed [7:0] final_out[255][24][24]
);

    // ----------------------------------------
    // Global parameters
    // ----------------------------------------
    localparam PIXEL_IN  = 8;
    localparam PIXEL_ACC = 32;

    // ----------------------------------------
    // Layer parameters (example first layers, extendable to 24)
    // ----------------------------------------
    // Layer 1: Conv 416x416x3 -> 416x416x16
    localparam CH_IN_L1  = 3;
    localparam CH_OUT_L1 = 16;
    localparam IMG_L1    = 416;
    localparam K_L1      = 3;
    localparam PAD_L1    = 1;
    localparam STR_L1    = 1;
    localparam OUT_L1    = 416;

    // Layer 2: MaxPool 416x416x16 -> 208x208x16
    localparam OUT_POOL2 = 208;

    // Layer 3: Conv 208x208x16 -> 208x208x32
    localparam CH_OUT_L3 = 32;
    localparam IMG_L3    = 208;
    localparam K_L3      = 3;
    localparam PAD_L3    = 1;
    localparam STR_L3    = 1;
    localparam OUT_L3    = 208;

    // Layer 4: MaxPool 208x208x32 -> 104x104x32
    localparam OUT_POOL4 = 104;

 // Layer 5: Conv 
    localparam CH_OUT_L5 = 64;
    localparam IMG_L5    = 104;
    localparam K_L5     = 3;
    localparam PAD_L5    = 1;
    localparam STR_L5   = 1;
    localparam OUT_L5    = 104;

    // Layer 6: MaxPool 
    localparam OUT_POOL6 = 52;

    // Layer 7: Conv 
    localparam CH_OUT_L7 = 128;
    localparam IMG_L7    = 52;
    localparam K_L7   = 3;
    localparam PAD_L7    = 1;
    localparam STR_L7  = 1;
    localparam OUT_L7    = 52;

 // Layer 8: MaxPool 
    localparam OUT_POOL8 = 26;
 // Layer 9: Conv 
    localparam CH_OUT_L9 = 256;
    localparam IMG_L9    = 26;
    localparam K_L9     = 3;
    localparam PAD_L9    = 1;
    localparam STR_L9  = 1;
    localparam OUT_L9    = 26;

    // Layer 10: MaxPool 
    localparam OUT_POOL10 = 13;

    // Layer 11: Conv 
    localparam CH_OUT_L11 = 512;
    localparam IMG_L11    = 13;
    localparam K_L11     = 3;
    localparam PAD_L11   = 1;
    localparam STR_L11 = 1;
    localparam OUT_L11    = 13;

     // Layer 12: MaxPool 
    localparam OUT_POOL12 = 12;
    localparam STR_L12 = 1 ;

    // Layer 13: Conv 
    localparam CH_OUT_L13 = 1024;
    localparam IMG_L13   = 12;
    localparam K_L13     = 3;
    localparam PAD_L13   = 1;
    localparam STR_L13 = 1;
    localparam OUT_L13    = 12;

    // Layer 14: Conv 
    localparam CH_OUT_L14 = 256;
    localparam IMG_L14   = 12;
    localparam K_L14     = 1;
    localparam PAD_L14   = 0;
    localparam STR_L14 = 1;
    localparam OUT_L14    = 12;

    // Layer 15: Conv 
    localparam CH_OUT_L15 = 512;
    localparam IMG_L15   = 12;
    localparam K_L15     = 3;
    localparam PAD_L15   = 1;
    localparam STR_L15 = 1;
    localparam OUT_L15    = 12;

    // Layer 16: Conv 
    localparam CH_OUT_L16 = 255;
    localparam IMG_L16   = 12;
    localparam K_L16     = 1;
    localparam PAD_L16   = 0;
    localparam STR_L16 = 1;
    localparam OUT_L16    = 12;

    // Layer 19: Conv 
    localparam CH_OUT_L19 = 128;
    localparam IMG_L19   = 12;
    localparam K_L19     = 1;
    localparam PAD_L19   = 0;
    localparam STR_L19 = 1;
    localparam OUT_L19    = 12;

    // Layer 20: Upsample
    localparam CH_UPL20 = 128;
    localparam IMG_L20   = 12;
    localparam FACTOR_L20 = 2 ;

    // Layer 22: Conv 
    localparam CH_OUT_L22 = 256;
    localparam IMG_L22  = 24;
    localparam K_L22    =  3;
    localparam PAD_L22  = 1;
    localparam STR_L22 = 1;
    localparam OUT_L22    = 24;

     // Layer 23: Conv 
    localparam CH_OUT_L23 = 255;
    localparam IMG_L23  = 24;
    localparam K_L23   =  1;
    localparam PAD_L23  = 0;
    localparam STR_L23 = 1;
    localparam OUT_L23    = 24;

    // ----------------------------------------
    // Feature map buffers (ping-pong style)
    // Extend this to cover all layers
    // ----------------------------------------
    logic signed [PIXEL_ACC-1:0] conv1_out [CH_OUT_L1][OUT_L1][OUT_L1];
    logic signed [PIXEL_ACC-1:0] relu1_out [CH_OUT_L1][OUT_L1][OUT_L1];
    logic signed [7:0]           quant1_out[CH_OUT_L1][OUT_L1][OUT_L1];
    logic signed [7:0]           pool2_out [CH_OUT_L1][OUT_POOL2][OUT_POOL2];

    logic signed [PIXEL_ACC-1:0] conv3_out [CH_OUT_L3][OUT_L3][OUT_L3];
    logic signed [PIXEL_ACC-1:0] relu3_out [CH_OUT_L3][OUT_L3][OUT_L3];
    logic signed [7:0]           quant3_out[CH_OUT_L3][OUT_L3][OUT_L3];
    logic signed [7:0]           pool4_out [CH_OUT_L3][OUT_POOL4][OUT_POOL4];

    logic signed [PIXEL_ACC-1:0] conv5_out [CH_OUT_L5][OUT_L5][OUT_L5];
    logic signed [PIXEL_ACC-1:0] relu5_out [CH_OUT_L5][OUT_L5][OUT_L5];
    logic signed [7:0]           quant5_out[CH_OUT_L5][OUT_L5][OUT_L5];
    logic signed [7:0]           pool6_out [CH_OUT_L5][OUT_POOL6][OUT_POOL6];

    logic signed [PIXEL_ACC-1:0] conv7_out [CH_OUT_L7][OUT_L7][OUT_L7];
    logic signed [PIXEL_ACC-1:0] relu7_out [CH_OUT_L7][OUT_L7][OUT_L7];
    logic signed [7:0]           quant7_out[CH_OUT_L7][OUT_L7][OUT_L7];
    logic signed [7:0]           pool8_out [CH_OUT_L7][OUT_POOL8][OUT_POOL8];
    
    logic signed [PIXEL_ACC-1:0] conv9_out [CH_OUT_L9][OUT_L9][OUT_L9];
    logic signed [PIXEL_ACC-1:0] relu9_out [CH_OUT_L9][OUT_L9][OUT_L9];
    logic signed [7:0]           quant9_out[CH_OUT_L9][OUT_L9][OUT_L9];
    logic signed [7:0]           pool10_out [CH_OUT_L9][OUT_POOL10][OUT_POOL10];

    logic signed [PIXEL_ACC-1:0] conv11_out [CH_OUT_L11][OUT_L11][OUT_L11];
    logic signed [PIXEL_ACC-1:0] relu11_out [CH_OUT_L11][OUT_L11][OUT_L11];
    logic signed [7:0]           quant11_out[CH_OUT_L11][OUT_L11][OUT_L11];
    logic signed [7:0]           pool12_out [CH_OUT_L11][OUT_POOL12][OUT_POOL12];

    logic signed [PIXEL_ACC-1:0] conv13_out [CH_OUT_L13][OUT_L13][OUT_L13];
    logic signed [PIXEL_ACC-1:0] relu13_out [CH_OUT_L13][OUT_L13][OUT_L13];
    logic signed [7:0]           quant13_out[CH_OUT_L13][OUT_L13][OUT_L13];
    
    logic signed [PIXEL_ACC-1:0] conv14_out [CH_OUT_L14][OUT_L14][OUT_L14];
    logic signed [PIXEL_ACC-1:0] relu14_out [CH_OUT_L14][OUT_L14][OUT_L14];
    logic signed [7:0]           quant14_out[CH_OUT_L14][OUT_L14][OUT_L14];

    logic signed [PIXEL_ACC-1:0] conv15_out [CH_OUT_L15][OUT_L15][OUT_L15];
    logic signed [PIXEL_ACC-1:0] relu15_out [CH_OUT_L15][OUT_L15][OUT_L15];
    logic signed [7:0]           quant15_out[CH_OUT_L15][OUT_L15][OUT_L15];

    logic signed [PIXEL_ACC-1:0] conv16_out [CH_OUT_L16][OUT_L16][OUT_L16];
    logic signed [PIXEL_ACC-1:0] relu16_out [CH_OUT_L16][OUT_L16][OUT_L16];
    logic signed [7:0]           quant16_out[CH_OUT_L16][OUT_L16][OUT_L16];

    logic signed [PIXEL_ACC-1:0] conv19_out [CH_OUT_L19][OUT_L19][OUT_L19];
    logic signed [PIXEL_ACC-1:0] relu19_out [CH_OUT_L19][OUT_L19][OUT_L19];
    logic signed [7:0]           quant19_out[CH_OUT_L19][OUT_L19][OUT_L19];
    logic signed [7:0]           upsample20_out [CH_OUT_L19][IMG_L20*FACTOR_L20][IMG_L20*FACTOR_L20];

    logic signed [PIXEL_ACC-1:0] conv22_out [CH_OUT_L22][OUT_L22][OUT_L22];
    logic signed [PIXEL_ACC-1:0] relu22_out [CH_OUT_L22][OUT_L22][OUT_L22];
    logic signed [7:0]           quant22_out[CH_OUT_L22][OUT_L22][OUT_L22];

    logic signed [PIXEL_ACC-1:0] conv23_out [CH_OUT_L23][OUT_L23][OUT_L23];
    logic signed [PIXEL_ACC-1:0] relu23_out [CH_OUT_L23][OUT_L23][OUT_L23];
    logic signed [7:0]           quant23_out[CH_OUT_L23][OUT_L23][OUT_L23];

    logic conv1_valid, relu1_valid , quant1_valid , pool2_valid,
          conv3_valid, relu3_valid , quant3_valid , pool4_valid,
          conv5_valid, relu5_valid , quant5_valid , pool6_valid ,
          conv7_valid , relu7_valid , quant7_valid , pool8_valid ,
          conv9_valid  , relu9_valid , quant9_valid , pool10_valid ,
          conv11_valid  ,relu11_valid , quant11_valid , pool12_valid ,
          conv13_valid  , relu13_valid , quant13_valid , 
          conv14_valid  , relu14_valid , quant14_valid , 
          conv15_valid  , relu15_valid , quant15_valid , 
          conv16_valid  , relu16_valid , quant16_valid , 
          conv19_valid  , relu19_valid , quant19_valid , 
          upsample20_valid  , conv22_valid  , relu22_valid , quant22_valid , 
          conv23_valid  , relu23_valid , quant23_valid ;

    logic conv1_done,relu1_done,quant1_done,pool2_done,conv3_done,relu3_done,quant3_done,pool4_done,
    conv5_done,relu5_done,quant5_done,pool6_done,conv7_done,relu7_done,quant7_done,pool8_done,conv9_done,
    relu9_done,quant9_done,pool10_done,conv11_done,relu11_done,quant11_done,pool12_done,conv13_done,
    relu13_done,quant13_done,conv14_done,relu14_done,quant14_done,conv15_done,relu15_done,quant15_done,
    conv16_done,relu16_done,quant16_done,conv19_done,relu19_done,quant19_done,upsample20_done,conv22_done,
    relu22_done,quant22_done,conv23_done,relu23_done,quant23_done;



    // ----------------------------------------
    // Weights and biases placeholders
    // Load all HEX files in one initial block
    // ----------------------------------------
    logic signed [PIXEL_IN-1:0] filter_l1[CH_OUT_L1][CH_IN_L1][K_L1][K_L1];
    logic signed [31:0] bias_l1[CH_OUT_L1];

    logic signed [PIXEL_IN-1:0] filter_l3[CH_OUT_L3][CH_OUT_L1][K_L3][K_L3];
    logic signed [31:0] bias_l3[CH_OUT_L3];

    logic signed [PIXEL_IN-1:0] filter_l5[CH_OUT_L5][CH_OUT_L3][K_L5][K_L5];
    logic signed [31:0] bias_l5[CH_OUT_L5];

    logic signed [PIXEL_IN-1:0] filter_l7[CH_OUT_L7][CH_OUT_L5][K_L7][K_L7];
    logic signed [31:0] bias_l7[CH_OUT_L7];

    logic signed [PIXEL_IN-1:0] filter_l9[CH_OUT_L9][CH_OUT_L7][K_L9][K_L9];
    logic signed [31:0] bias_l9[CH_OUT_L9];

    logic signed [PIXEL_IN-1:0] filter_l11[CH_OUT_L11][CH_OUT_L9][K_L11][K_L11];
    logic signed [31:0] bias_l11[CH_OUT_L11];

    logic signed [PIXEL_IN-1:0] filter_l13[CH_OUT_L13][CH_OUT_L11][K_L13][K_L13];
    logic signed [31:0] bias_l13[CH_OUT_L13];

    logic signed [PIXEL_IN-1:0] filter_l14[CH_OUT_L14][CH_OUT_L13][K_L14][K_L14];
    logic signed [31:0] bias_l14[CH_OUT_L14];

    logic signed [PIXEL_IN-1:0] filter_l15[CH_OUT_L15][CH_OUT_L14][K_L15][K_L15];
    logic signed [31:0] bias_l15[CH_OUT_L15];

    logic signed [PIXEL_IN-1:0] filter_l16[CH_OUT_L16][CH_OUT_L15][K_L16][K_L16];
    logic signed [31:0] bias_l16[CH_OUT_L16];

    logic signed [PIXEL_IN-1:0] filter_l19[CH_OUT_L19][CH_OUT_L16][K_L19][K_L19];
    logic signed [31:0] bias_l19[CH_OUT_L19];

    logic signed [PIXEL_IN-1:0] filter_l22[CH_OUT_L22][CH_OUT_L19][K_L22][K_L22];
    logic signed [31:0] bias_l22[CH_OUT_L22];

    logic signed [PIXEL_IN-1:0] filter_l23[CH_OUT_L23][CH_OUT_L22][K_L23][K_L23];
    logic signed [31:0] bias_l23[CH_OUT_L23];



// -----          final output ------------------------
genvar c,i,j;
generate
for (c=0; c<CH_OUT_L23; c++)
for (i=0; i<OUT_L23; i++)
for (j=0; j<OUT_L23; j++)
    assign final_out[c][i][j] = quant23_out[c][i][j];
endgenerate
// ---------------------



   initial begin
        int idx, co, ci, m, n;

        // --------------------------
        // Layer 1
        logic signed [PIXEL_IN-1:0] filter_rom_l1 [0:CH_OUT_L1*CH_IN_L1*K_L1*K_L1-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l1 [0:CH_OUT_L1-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l3 [0:CH_OUT_L3*CH_OUT_L1*K_L3*K_L3-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l3 [0:CH_OUT_L3-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l5 [0:CH_OUT_L5*CH_OUT_L3*K_L5*K_L5-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l5 [0:CH_OUT_L5-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l7 [0:CH_OUT_L7*CH_OUT_L5*K_L7*K_L7-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l7 [0:CH_OUT_L7-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l9 [0:CH_OUT_L9*CH_OUT_L7*K_L9*K_L9-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l9 [0:CH_OUT_L9-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l11 [0:CH_OUT_L11*CH_OUT_L9*K_L11*K_L11-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l11 [0:CH_OUT_L11-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l13 [0:CH_OUT_L13*CH_OUT_L11*K_L13*K_L13-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l13 [0:CH_OUT_L13-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l14 [0:CH_OUT_L14*CH_OUT_L13*K_L14*K_L14-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l14 [0:CH_OUT_L14-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l15 [0:CH_OUT_L15*CH_OUT_L14*K_L15*K_L15-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l15 [0:CH_OUT_L15-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l16 [0:CH_OUT_L16*CH_OUT_L15*K_L16*K_L16-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l16 [0:CH_OUT_L16-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l19 [0:CH_OUT_L19*CH_OUT_L16*K_L19*K_L19-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l19 [0:CH_OUT_L19-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l22 [0:CH_OUT_L22*CH_OUT_L19*K_L22*K_L22-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l22 [0:CH_OUT_L22-1];
        logic signed [PIXEL_IN-1:0] filter_rom_l23 [0:CH_OUT_L23*CH_OUT_L22*K_L23*K_L23-1];
        logic signed [PIXEL_ACC-1:0] bias_rom_l23 [0:CH_OUT_L23-1];
        


        $readmemh("layer0_conv_weights.hex", filter_rom_l1);
        $readmemh("layer0_conv_bias.hex", bias_rom_l1);
        idx = 0;
        for (co=0; co<CH_OUT_L1; co++)
            for (ci=0; ci<CH_IN_L1; ci++)
                for (m=0; m<K_L1; m++)
                    for (n=0; n<K_L1; n++)
                        filter_l1[co][ci][m][n] = filter_rom_l1[idx++];
        for (co=0; co<CH_OUT_L1; co++)      //------> line 298 is here
            bias_l1[co] = bias_rom_l1[co];

        // --------------------------
        // Layer 3 (next conv)
        
        $readmemh("layer2_conv_weights.hex", filter_rom_l3);
        $readmemh("layer2_conv_bias.hex", bias_rom_l3);
        idx = 0;
        for (co=0; co<CH_OUT_L3; co++)
            for (ci=0; ci<CH_OUT_L1; ci++)
                for (m=0; m<K_L3; m++)
                    for (n=0; n<K_L3; n++)
                        filter_l3[co][ci][m][n] = filter_rom_l3[idx++];
        for (co=0; co<CH_OUT_L3; co++)
            bias_l3[co] = bias_rom_l3[co];
        
        // --------------------------
        // Layer 5 (next conv)
        
        $readmemh("layer4_conv_weights.hex", filter_rom_l5);
        $readmemh("layer4_conv_bias.hex", bias_rom_l5);
        idx = 0;
        for (co=0; co<CH_OUT_L5; co++)
            for (ci=0; ci<CH_OUT_L3; ci++)
                for (m=0; m<K_L5; m++)
                    for (n=0; n<K_L5; n++)
                        filter_l5[co][ci][m][n] = filter_rom_l5[idx++];
        for (co=0; co<CH_OUT_L5; co++)
            bias_l5[co] = bias_rom_l5[co];

        // --------------------------
        // Layer 7 (next conv)
        
        $readmemh("layer6_conv_weights.hex", filter_rom_l7);
        $readmemh("layer6_conv_bias.hex", bias_rom_l7);
        idx = 0;
        for (co=0; co<CH_OUT_L7; co++)
            for (ci=0; ci<CH_OUT_L5; ci++)
                for (m=0; m<K_L7; m++)
                    for (n=0; n<K_L7; n++)
                        filter_l7[co][ci][m][n] = filter_rom_l7[idx++];
        for (co=0; co<CH_OUT_L7; co++)
            bias_l7[co] = bias_rom_l7[co];
        
        // --------------------------
        // Layer 9 (next conv)
        
        $readmemh("layer8_conv_weights.hex", filter_rom_l9);
        $readmemh("layer8_conv_bias.hex", bias_rom_l9);
        idx = 0;
        for (co=0; co<CH_OUT_L9; co++)
            for (ci=0; ci<CH_OUT_L7; ci++)
                for (m=0; m<K_L9; m++)
                    for (n=0; n<K_L9; n++)
                        filter_l9[co][ci][m][n] = filter_rom_l9[idx++];
        for (co=0; co<CH_OUT_L9; co++)
            bias_l9[co] = bias_rom_l9[co];
        
        // --------------------------
        // Layer 11 (next conv)
        
        $readmemh("layer10_conv_weights.hex", filter_rom_l11);
        $readmemh("layer10_conv_bias.hex", bias_rom_l11);
        idx = 0;
        for (co=0; co<CH_OUT_L11; co++)
            for (ci=0; ci<CH_OUT_L9; ci++)
                for (m=0; m<K_L11; m++)
                    for (n=0; n<K_L11; n++)
                        filter_l11[co][ci][m][n] = filter_rom_l11[idx++];
        for (co=0; co<CH_OUT_L11; co++)
            bias_l11[co] = bias_rom_l11[co];
        
        // --------------------------
        // Layer 13 (next conv)
        
        $readmemh("layer12_conv_weights.hex", filter_rom_l13);
        $readmemh("layer12_conv_bias.hex", bias_rom_l13);
        idx = 0;
        for (co=0; co<CH_OUT_L13; co++)
            for (ci=0; ci<CH_OUT_L11; ci++)
                for (m=0; m<K_L13; m++)
                    for (n=0; n<K_L13; n++)
                        filter_l13[co][ci][m][n] = filter_rom_l13[idx++];
        for (co=0; co<CH_OUT_L13; co++)
            bias_l13[co] = bias_rom_l13[co];
        
        // --------------------------
        // Layer 14 (next conv)
        
        $readmemh("layer13_conv_weights.hex", filter_rom_l14);
        $readmemh("layer13_conv_bias.hex", bias_rom_l14);
        idx = 0;
        for (co=0; co<CH_OUT_L14; co++)
            for (ci=0; ci<CH_OUT_L13; ci++)
                for (m=0; m<K_L14; m++)
                    for (n=0; n<K_L14; n++)
                        filter_l14[co][ci][m][n] = filter_rom_l14[idx++];
        for (co=0; co<CH_OUT_L14; co++)
            bias_l14[co] = bias_rom_l14[co];
        
        // --------------------------
        // Layer 15 (next conv)
        
        $readmemh("layer14_conv_weights.hex", filter_rom_l15);
        $readmemh("layer14_conv_bias.hex", bias_rom_l15);
        idx = 0;
        for (co=0; co<CH_OUT_L15; co++)
            for (ci=0; ci<CH_OUT_L14; ci++)
                for (m=0; m<K_L15; m++)
                    for (n=0; n<K_L15; n++)
                        filter_l15[co][ci][m][n] = filter_rom_l15[idx++];
        for (co=0; co<CH_OUT_L15; co++)
            bias_l15[co] = bias_rom_l15[co];
        
        // --------------------------
        // Layer 16 (next conv)
        
        $readmemh("layer15_conv_weights.hex", filter_rom_l16);
        $readmemh("layer15_conv_bias.hex", bias_rom_l16);
        idx = 0;
        for (co=0; co<CH_OUT_L16; co++)
            for (ci=0; ci<CH_OUT_L15; ci++)
                for (m=0; m<K_L16; m++)
                    for (n=0; n<K_L16; n++)
                        filter_l16[co][ci][m][n] = filter_rom_l16[idx++];
        for (co=0; co<CH_OUT_L16; co++)
            bias_l16[co] = bias_rom_l16[co];

        // --------------------------
        // Layer 19 (next conv)
        
        $readmemh("layer16_conv_weights.hex", filter_rom_l19);
        $readmemh("layer16_conv_bias.hex", bias_rom_l19);
        idx = 0;
        for (co=0; co<CH_OUT_L19; co++)
            for (ci=0; ci<CH_OUT_L16; ci++)
                for (m=0; m<K_L19; m++)
                    for (n=0; n<K_L19; n++)
                        filter_l19[co][ci][m][n] = filter_rom_l19[idx++];
        for (co=0; co<CH_OUT_L19; co++)
            bias_l19[co] = bias_rom_l19[co];

        // --------------------------
        // Layer 22 (next conv)
        
        $readmemh("layer17_conv_weights.hex", filter_rom_l22);
        $readmemh("layer17_conv_bias.hex", bias_rom_l22);
        idx = 0;
        for (co=0; co<CH_OUT_L22; co++)
            for (ci=0; ci<CH_OUT_L19; ci++)
                for (m=0; m<K_L22; m++)
                    for (n=0; n<K_L22; n++)
                        filter_l22[co][ci][m][n] = filter_rom_l22[idx++];
        for (co=0; co<CH_OUT_L22; co++)
            bias_l22[co] = bias_rom_l22[co];
        
        // --------------------------
        // Layer 23 (next conv)
       
        $readmemh("layer18_conv_weights.hex", filter_rom_l23);
        $readmemh("layer18_conv_bias.hex", bias_rom_l23);
        idx = 0;
        for (co=0; co<CH_OUT_L23; co++)
            for (ci=0; ci<CH_OUT_L22; ci++)
                for (m=0; m<K_L23; m++)
                    for (n=0; n<K_L23; n++)
                        filter_l23[co][ci][m][n] = filter_rom_l23[idx++];
        for (co=0; co<CH_OUT_L23; co++)
            bias_l23[co] = bias_rom_l23[co];
        // --------------------------
        // Repeat the same pattern for remaining conv layers up to layer 24
    end

    // ----------------------------------------
    // FSM controller
    // ----------------------------------------
    typedef enum logic [6:0] {
        S_IDLE,

        S_CONV1, S_WAIT_CONV1,
        S_RELU1, S_WAIT_RELU1,
        S_QUANT1, S_WAIT_QUANT1,
        S_POOL2, S_WAIT_POOL2,

        S_CONV3, S_WAIT_CONV3,
        S_RELU3, S_WAIT_RELU3,
        S_QUANT3, S_WAIT_QUANT3,
        S_POOL4, S_WAIT_POOL4,

        S_CONV5, S_WAIT_CONV5,
        S_RELU5, S_WAIT_RELU5,
        S_QUANT5, S_WAIT_QUANT5,
        S_POOL6, S_WAIT_POOL6,

        S_CONV7, S_WAIT_CONV7,
        S_RELU7, S_WAIT_RELU7,
        S_QUANT7, S_WAIT_QUANT7,
        S_POOL8, S_WAIT_POOL8,

        S_CONV9, S_WAIT_CONV9,
        S_RELU9, S_WAIT_RELU9,
        S_QUANT9, S_WAIT_QUANT9,
        S_POOL10, S_WAIT_POOL10,

        S_CONV11, S_WAIT_CONV11,
        S_RELU11, S_WAIT_RELU11,
        S_QUANT11, S_WAIT_QUANT11,
        S_POOL12, S_WAIT_POOL12,

        S_CONV13, S_WAIT_CONV13,
        S_RELU13, S_WAIT_RELU13,
        S_QUANT13, S_WAIT_QUANT13,
        
        S_CONV14, S_WAIT_CONV14,
        S_RELU14, S_WAIT_RELU14,
        S_QUANT14, S_WAIT_QUANT14,
        
        S_CONV15, S_WAIT_CONV15,
        S_RELU15, S_WAIT_RELU15,
        S_QUANT15, S_WAIT_QUANT15,

        S_CONV16, S_WAIT_CONV16,
        S_RELU16, S_WAIT_RELU16,
        S_QUANT16, S_WAIT_QUANT16,

        S_CONV19, S_WAIT_CONV19,
        S_RELU19, S_WAIT_RELU19,
        S_QUANT19, S_WAIT_QUANT19,

        S_UPSAMPLE_20, S_WAIT_UPSAMPLE_20,

        S_CONV22, S_WAIT_CONV22,
        S_RELU22, S_WAIT_RELU22,
        S_QUANT22, S_WAIT_QUANT22,

        S_CONV23, S_WAIT_CONV23,
        S_RELU23, S_WAIT_RELU23,
        S_QUANT23, S_WAIT_QUANT23,

        S_DONE
    } state_t;

    state_t state, next_state;

    // FSM state update
    always_ff @(posedge clk) begin
        if (rst)
            state <= S_IDLE;
        else
            state <= next_state;
    end

    //---------------------------------------------------------------------------------------------------------
 //---------------------------------------------------------------------------------------------------------
// FSM next state logic
     //---------------------------------------------------------------------------------------------------------
 //---------------------------------------------------------------------------------------------------------
 //---------------------------------------------------------------------------------------------------------

    always_comb begin
        next_state = state;

        case (state)
            S_IDLE: if (in_valid) next_state = S_CONV1;

            S_CONV1: next_state = S_WAIT_CONV1;
            S_WAIT_CONV1: if (conv1_done) next_state = S_RELU1;

            S_RELU1: next_state = S_WAIT_RELU1;
            S_WAIT_RELU1: if (relu1_done) next_state = S_QUANT1;

            S_QUANT1: next_state = S_WAIT_QUANT1;
            S_WAIT_QUANT1: if (quant1_done) next_state = S_POOL2;

            S_POOL2: next_state = S_WAIT_POOL2;
            S_WAIT_POOL2: if (pool2_done) next_state = S_CONV3;

            S_CONV3: next_state = S_WAIT_CONV3;
            S_WAIT_CONV3: if (conv3_done) next_state = S_RELU3;

            S_RELU3: next_state = S_WAIT_RELU3;
            S_WAIT_RELU3: if (relu3_done) next_state = S_QUANT3;

            S_QUANT3: next_state = S_WAIT_QUANT3;
            S_WAIT_QUANT3: if (quant3_done) next_state = S_POOL4;

            S_POOL4: next_state = S_WAIT_POOL4;
            S_WAIT_POOL4: if (pool4_done) next_state = S_CONV5;

            S_CONV5: next_state = S_WAIT_CONV5;
            S_WAIT_CONV5: if (conv5_done) next_state = S_RELU5;

            S_RELU5: next_state = S_WAIT_RELU5;
            S_WAIT_RELU5: if (relu5_done) next_state = S_QUANT5;

            S_QUANT5: next_state = S_WAIT_QUANT5;
            S_WAIT_QUANT5: if (quant5_done) next_state = S_POOL6;

            S_POOL6: next_state = S_WAIT_POOL6;
            S_WAIT_POOL6: if (pool6_done) next_state = S_CONV7;

            S_CONV7: next_state = S_WAIT_CONV7;
            S_WAIT_CONV7: if (conv7_done) next_state = S_RELU7;

            S_RELU7: next_state = S_WAIT_RELU7;
            S_WAIT_RELU7: if (relu7_done) next_state = S_QUANT7;

            S_QUANT7: next_state = S_WAIT_QUANT7;
            S_WAIT_QUANT7: if (quant7_done) next_state = S_POOL8;
            
            S_POOL8: next_state = S_WAIT_POOL8;
            S_WAIT_POOL8: if (pool8_done) next_state = S_CONV9;

            S_CONV9: next_state = S_WAIT_CONV9;
            S_WAIT_CONV9: if (conv9_done) next_state = S_RELU9;

            S_RELU9: next_state = S_WAIT_RELU9;
            S_WAIT_RELU9: if (relu9_done) next_state = S_QUANT9;

            S_QUANT9: next_state = S_WAIT_QUANT9;
            S_WAIT_QUANT9: if (quant9_done) next_state = S_POOL10;

            S_POOL10: next_state = S_WAIT_POOL10;
            S_WAIT_POOL10: if (pool10_done) next_state = S_CONV11;

            S_CONV11: next_state = S_WAIT_CONV11;
            S_WAIT_CONV11: if (conv11_done) next_state = S_RELU11;

            S_RELU11: next_state = S_WAIT_RELU11;
            S_WAIT_RELU11: if (relu11_done) next_state = S_QUANT11;

            S_QUANT11: next_state = S_WAIT_QUANT11;
            S_WAIT_QUANT11: if (quant11_done) next_state = S_POOL12;

            S_POOL12: next_state = S_WAIT_POOL12;
            S_WAIT_POOL12: if (pool12_done) next_state = S_CONV13;

            S_CONV13: next_state = S_WAIT_CONV13;
            S_WAIT_CONV13: if (conv13_done) next_state = S_RELU13;

            S_RELU13: next_state = S_WAIT_RELU13;
            S_WAIT_RELU13: if (relu13_done) next_state = S_QUANT13;

            S_QUANT13: next_state = S_WAIT_QUANT13;
            S_WAIT_QUANT13: if (quant13_done) next_state = S_CONV14;

            S_CONV14: next_state = S_WAIT_CONV14;
            S_WAIT_CONV14: if (conv14_done) next_state = S_RELU14;

            S_RELU14: next_state = S_WAIT_RELU14;
            S_WAIT_RELU14: if (relu14_done) next_state = S_QUANT14;

            S_QUANT14: next_state = S_WAIT_QUANT14;
            S_WAIT_QUANT14: if (quant14_done) next_state = S_CONV15;

            S_CONV15: next_state = S_WAIT_CONV15;
            S_WAIT_CONV15: if (conv15_done) next_state = S_RELU15;

            S_RELU15: next_state = S_WAIT_RELU15;
            S_WAIT_RELU15: if (relu15_done) next_state = S_QUANT15;

            S_QUANT15: next_state = S_WAIT_QUANT15;
            S_WAIT_QUANT15: if (quant15_done) next_state = S_CONV16;

            S_CONV16: next_state = S_WAIT_CONV16;
            S_WAIT_CONV16: if (conv16_done) next_state = S_RELU16;

            S_RELU16: next_state = S_WAIT_RELU16;
            S_WAIT_RELU16: if (relu16_done) next_state = S_QUANT16;

            S_QUANT16: next_state = S_WAIT_QUANT16;
            S_WAIT_QUANT16: if (quant16_done) next_state = S_CONV19;

            S_CONV19: next_state = S_WAIT_CONV19;
            S_WAIT_CONV19: if (conv19_done) next_state = S_RELU19;

            S_RELU19: next_state = S_WAIT_RELU19;
            S_WAIT_RELU19: if (relu19_done) next_state = S_QUANT19;

            S_QUANT19: next_state = S_WAIT_QUANT19;
            S_WAIT_QUANT19: if (quant19_done) next_state = S_UPSAMPLE_20;

            S_UPSAMPLE_20: next_state = S_WAIT_UPSAMPLE_20;
            S_WAIT_UPSAMPLE_20: if (upsample20_done) next_state = S_CONV22;

            S_CONV22: next_state = S_WAIT_CONV22;
            S_WAIT_CONV22: if (conv22_done) next_state = S_RELU22;

            S_RELU22: next_state = S_WAIT_RELU22;
            S_WAIT_RELU22: if (relu22_done) next_state = S_QUANT22;

            S_QUANT22: next_state = S_WAIT_QUANT22;
            S_WAIT_QUANT22: if (quant22_done) next_state = S_CONV23 ;

            S_CONV23: next_state = S_WAIT_CONV23;
            S_WAIT_CONV23: if (conv23_done) next_state = S_RELU23;

            S_RELU23: next_state = S_WAIT_RELU23;
            S_WAIT_RELU23: if (relu23_done) next_state = S_QUANT23;

            S_QUANT23: next_state = S_WAIT_QUANT23;
            S_WAIT_QUANT23: if (quant23_done) next_state = S_DONE ;
            
            S_DONE: next_state = S_DONE;
        endcase
    end

    // FSM outputs
    always_comb begin
        conv1_valid  = 0; relu1_valid = 0; quant1_valid = 0; pool2_valid = 0;
        conv3_valid  = 0; relu3_valid = 0; quant3_valid = 0; pool4_valid = 0;
        conv5_valid  = 0; relu5_valid = 0; quant5_valid = 0; pool6_valid = 0;
        conv7_valid  = 0; relu7_valid = 0; quant7_valid = 0; pool8_valid = 0;
        conv9_valid  = 0; relu9_valid = 0; quant9_valid = 0; pool10_valid = 0;
        conv11_valid  = 0;relu11_valid = 0; quant11_valid = 0; pool12_valid = 0;
        conv13_valid  = 0; relu13_valid = 0; quant13_valid = 0; 
        conv14_valid  = 0; relu14_valid = 0; quant14_valid = 0; 
        conv15_valid  = 0; relu15_valid = 0; quant15_valid = 0; 
        conv16_valid  = 0; relu16_valid = 0; quant16_valid = 0; 
        conv19_valid  = 0; relu19_valid = 0; quant19_valid = 0; 
        upsample20_valid  = 0; 
        conv22_valid  = 0; relu22_valid = 0; quant22_valid = 0; 
        conv23_valid  = 0; relu23_valid = 0; quant23_valid = 0;
        done = 0;

        case (state)
            S_CONV1:  conv1_valid  = 1;
            S_RELU1:  relu1_valid  = 1;
            S_QUANT1: quant1_valid = 1;
            S_POOL2:  pool2_valid  = 1;

            S_CONV3:  conv3_valid  = 1;
            S_RELU3:  relu3_valid  = 1;
            S_QUANT3: quant3_valid = 1;
            S_POOL4:  pool4_valid  = 1;

            S_CONV5:  conv5_valid  = 1;
            S_RELU5:  relu5_valid  = 1;
            S_QUANT5: quant5_valid = 1;
            S_POOL6:  pool6_valid  = 1;

            S_CONV7:  conv7_valid  = 1;
            S_RELU7:  relu7_valid  = 1;
            S_QUANT7: quant7_valid = 1;
            S_POOL8:  pool8_valid  = 1;

            S_CONV9:  conv9_valid  = 1;
            S_RELU9:  relu9_valid  = 1;
            S_QUANT9: quant9_valid = 1;
            S_POOL10:  pool10_valid  = 1;

            S_CONV11:  conv11_valid  = 1;
            S_RELU11:  relu11_valid  = 1;
            S_QUANT11: quant11_valid = 1;
            S_POOL12:  pool12_valid  = 1;

            S_CONV13:  conv13_valid  = 1;
            S_RELU13:  relu13_valid  = 1;
            S_QUANT13: quant13_valid = 1;

            S_CONV14:  conv14_valid  = 1;
            S_RELU14:  relu14_valid  = 1;
            S_QUANT14: quant14_valid = 1;

            S_CONV15:  conv15_valid  = 1;
            S_RELU15:  relu15_valid  = 1;
            S_QUANT15: quant15_valid = 1;

            S_CONV16:  conv16_valid  = 1;
            S_RELU16:  relu16_valid  = 1;
            S_QUANT16: quant16_valid = 1;

            S_CONV19:  conv19_valid  = 1;
            S_RELU19:  relu19_valid  = 1;
            S_QUANT19: quant19_valid = 1;

            S_UPSAMPLE_20: upsample20_valid = 1 ;

            S_CONV22:  conv22_valid  = 1;
            S_RELU22:  relu22_valid  = 1;
            S_QUANT22: quant22_valid = 1;

            S_CONV23:  conv23_valid  = 1;
            S_RELU23:  relu23_valid  = 1;
            S_QUANT23: quant23_valid = 1;

            S_DONE:   done = 1;
        endcase
    end

    // ----------------------------------------
    // CNN blocks instantiation
    // ----------------------------------------
    // Layer 1
    conv #(
        .CH_IN(CH_IN_L1), .CH_OUT(CH_OUT_L1),
        .IMG_SIZE(IMG_L1), .K(K_L1),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L1), .STRIDE(STR_L1)
    ) conv1 (
        .clk(clk), .rst(rst),
        .in_valid(conv1_valid),
        .image(image_in),
        .filter(filter_l1),
        .bias(bias_l1),
        .op_img(conv1_out),
        .out_valid(),
        .done(conv1_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L1), .IMG_SIZE(OUT_L1), .DATA_W(PIXEL_ACC)
    ) relu1 (
        .clk(clk), .rst(rst),
        .in_valid(relu1_valid),
        .in_map(conv1_out),
        .out_map(relu1_out),
        .out_valid(), //--this line 852
        .done(relu1_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L1), .IMG_SIZE(OUT_L1), .SHIFT(8)
    ) quant1 (
        .clk(clk), .rst(rst),
        .in_valid(quant1_valid),
        .in_map(relu1_out),
        .out_map(quant1_out),
        .out_valid(), //this is line 864
        .done(quant1_done)
    );

//layer 2------
     max_pooling #(
        .IMG_SIZE(OUT_L1), .CH_IN(CH_OUT_L1), .CH_OUT(CH_OUT_L1),
        .K(2), .STRIDE(2)
    ) pool2 (
        .clk(clk), .rst(rst),
        .in_valid(pool2_valid),
        .x(quant1_out),
        .pooled_output(pool2_out),
        .out_valid(), //-->this is line 877
        .done(pool2_done)
    );

    // Layer 3
       conv #(
        .CH_IN(CH_OUT_L1), .CH_OUT(CH_OUT_L3),
        .IMG_SIZE(IMG_L3), .K(K_L3),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L3), .STRIDE(STR_L3)
    ) conv3 (
        .clk(clk), .rst(rst),
        .in_valid(conv3_valid),
        .image(pool2_out),
        .filter(filter_l3),
        .bias(bias_l3),
        .op_img(conv3_out),
        .out_valid(),
        .done(conv3_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L3), .IMG_SIZE(OUT_L3), .DATA_W(PIXEL_ACC)
    ) relu3 (
        .clk(clk), .rst(rst),
        .in_valid(relu3_valid),
        .in_map(conv3_out),
        .out_map(relu3_out),
        .out_valid(),
        .done(relu3_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L3), .IMG_SIZE(OUT_L3), .SHIFT(8)
    ) quant3 (
        .clk(clk), .rst(rst),
        .in_valid(quant3_valid),
        .in_map(relu3_out),
        .out_map(quant3_out),
        .out_valid(),
        .done(quant3_done)
    );

//layer 4   ------
     max_pooling #(
        .IMG_SIZE(OUT_L3), .CH_IN(CH_OUT_L3), .CH_OUT(CH_OUT_L3),
        .K(2), .STRIDE(2)
    ) pool4 (
        .clk(clk), .rst(rst),
        .in_valid(pool4_valid),
        .x(quant3_out),
        .pooled_output(pool4_out),
        .out_valid(),
        .done(pool4_done)
    );

    // Layer 5
       conv #(
        .CH_IN(CH_OUT_L3), .CH_OUT(CH_OUT_L5),
        .IMG_SIZE(IMG_L5), .K(K_L5),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L5), .STRIDE(STR_L5)
    ) conv5 (
        .clk(clk), .rst(rst),
        .in_valid(conv5_valid),
        .image(pool4_out),
        .filter(filter_l5),
        .bias(bias_l5),
        .op_img(conv5_out),
        .out_valid(),
        .done(conv5_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L5), .IMG_SIZE(OUT_L5), .DATA_W(PIXEL_ACC)
    ) relu5 (
        .clk(clk), .rst(rst),
        .in_valid(relu5_valid),
        .in_map(conv5_out),
        .out_map(relu5_out),
        .out_valid(),
        .done(relu5_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L5), .IMG_SIZE(OUT_L5), .SHIFT(8)
    ) quant5 (
        .clk(clk), .rst(rst),
        .in_valid(quant5_valid),
        .in_map(relu5_out),
        .out_map(quant5_out),
        .out_valid(),
        .done(quant5_done)
    );

//layer 6   ------
     max_pooling #(
        .IMG_SIZE(OUT_L5), .CH_IN(CH_OUT_L5), .CH_OUT(CH_OUT_L5),
        .K(2), .STRIDE(2)
    ) pool6 (
        .clk(clk), .rst(rst),
        .in_valid(pool6_valid),
        .x(quant5_out),
        .pooled_output(pool6_out),
        .out_valid(),
        .done(pool6_done)
    );

 // Layer 7
       conv #(
        .CH_IN(CH_OUT_L5), .CH_OUT(CH_OUT_L7),
        .IMG_SIZE(IMG_L7), .K(K_L7),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L7), .STRIDE(STR_L7)
    ) conv7 (
        .clk(clk), .rst(rst),
        .in_valid(conv7_valid),
        .image(pool6_out),
        .filter(filter_l7),
        .bias(bias_l7),
        .op_img(conv7_out),
        .out_valid(),
        .done(conv7_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L7), .IMG_SIZE(OUT_L7), .DATA_W(PIXEL_ACC)
    ) relu7 (
        .clk(clk), .rst(rst),
        .in_valid(relu7_valid),
        .in_map(conv7_out),
        .out_map(relu7_out),
        .out_valid(),
        .done(relu7_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L7), .IMG_SIZE(OUT_L7), .SHIFT(8)
    ) quant7 (
        .clk(clk), .rst(rst),
        .in_valid(quant7_valid),
        .in_map(relu7_out),
        .out_map(quant7_out),
        .out_valid(),
        .done(quant7_done)
    );

//layer 8   ------
     max_pooling #(
        .IMG_SIZE(OUT_L7), .CH_IN(CH_OUT_L7), .CH_OUT(CH_OUT_L7),
        .K(2), .STRIDE(2)
    ) pool8 (
        .clk(clk), .rst(rst),
        .in_valid(pool8_valid),
        .x(quant7_out),
        .pooled_output(pool8_out),
        .out_valid(),
        .done(pool8_done)
    );

//layer 9 -----
       conv #(
        .CH_IN(CH_OUT_L7), .CH_OUT(CH_OUT_L9),
        .IMG_SIZE(IMG_L9), .K(K_L9),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L9), .STRIDE(STR_L9)
    ) conv9 (
        .clk(clk), .rst(rst),
        .in_valid(conv9_valid),
        .image(pool8_out),
        .filter(filter_l9),
        .bias(bias_l9),
        .op_img(conv9_out),
        .out_valid(),
        .done(conv9_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L9), .IMG_SIZE(OUT_L9), .DATA_W(PIXEL_ACC)
    ) relu9 (
        .clk(clk), .rst(rst),
        .in_valid(relu9_valid),
        .in_map(conv9_out),
        .out_map(relu9_out),
        .out_valid(),
        .done(relu9_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L9), .IMG_SIZE(OUT_L9), .SHIFT(8)
    ) quant9 (
        .clk(clk), .rst(rst),
        .in_valid(quant9_valid),
        .in_map(relu9_out),
        .out_map(quant9_out),
        .out_valid(),
        .done(quant9_done)
    );

//layer 10   ------
     max_pooling #(
        .IMG_SIZE(OUT_L9), .CH_IN(CH_OUT_L9), .CH_OUT(CH_OUT_L9),
        .K(2), .STRIDE(2)
    ) pool10 (
        .clk(clk), .rst(rst),
        .in_valid(pool10_valid),
        .x(quant9_out),
        .pooled_output(pool10_out),
        .out_valid(),
        .done(pool10_done)
    );

// Layer 11
       conv #(
        .CH_IN(CH_OUT_L9), .CH_OUT(CH_OUT_L11),
        .IMG_SIZE(IMG_L11), .K(K_L11),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L11), .STRIDE(STR_L11)
    ) conv11 (
        .clk(clk), .rst(rst),
        .in_valid(conv11_valid),
        .image(pool10_out),
        .filter(filter_l11),
        .bias(bias_l11),
        .op_img(conv11_out),
        .out_valid(),
        .done(conv11_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L11), .IMG_SIZE(OUT_L11), .DATA_W(PIXEL_ACC)
    ) relu11 (
        .clk(clk), .rst(rst),
        .in_valid(relu11_valid),
        .in_map(conv11_out),
        .out_map(relu11_out),
        .out_valid(),
        .done(relu11_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L11), .IMG_SIZE(OUT_L11), .SHIFT(8)
    ) quant11 (
        .clk(clk), .rst(rst),
        .in_valid(quant11_valid),
        .in_map(relu11_out),
        .out_map(quant11_out),
        .out_valid(),
        .done(quant11_done)
    );

//layer 12   ------
     max_pooling #(
        .IMG_SIZE(OUT_L11), .CH_IN(CH_OUT_L11), .CH_OUT(CH_OUT_L11),
        .K(2), .STRIDE(1)
    ) pool12 (
        .clk(clk), .rst(rst),
        .in_valid(pool12_valid),
        .x(quant11_out),
        .pooled_output(pool12_out),
        .out_valid(),
        .done(pool12_done)
    );

// Layer 13
       conv #(
        .CH_IN(CH_OUT_L11), .CH_OUT(CH_OUT_L13),
        .IMG_SIZE(IMG_L13), .K(K_L13),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L13), .STRIDE(STR_L13)
    ) conv13 (
        .clk(clk), .rst(rst),
        .in_valid(conv13_valid),
        .image(pool12_out),
        .filter(filter_l13),
        .bias(bias_l13),
        .op_img(conv13_out),
        .out_valid(),
        .done(conv13_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L13), .IMG_SIZE(OUT_L13), .DATA_W(PIXEL_ACC)
    ) relu13 (
        .clk(clk), .rst(rst),
        .in_valid(relu13_valid),
        .in_map(conv13_out),
        .out_map(relu13_out),
        .out_valid(),
        .done(relu13_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L13), .IMG_SIZE(OUT_L13), .SHIFT(8)
    ) quant13 (
        .clk(clk), .rst(rst),
        .in_valid(quant13_valid),
        .in_map(relu13_out),
        .out_map(quant13_out),
        .out_valid(),
        .done(quant13_done)
    );

// Layer 14
       conv #(
        .CH_IN(CH_OUT_L13), .CH_OUT(CH_OUT_L14),
        .IMG_SIZE(IMG_L14), .K(K_L14),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L14), .STRIDE(STR_L14)
    ) conv14 (
        .clk(clk), .rst(rst),
        .in_valid(conv14_valid),
        .image(quant13_out),
        .filter(filter_l14),
        .bias(bias_l14),
        .op_img(conv14_out),
        .out_valid(),
        .done(conv14_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L14), .IMG_SIZE(OUT_L14), .DATA_W(PIXEL_ACC)
    ) relu14 (
        .clk(clk), .rst(rst),
        .in_valid(relu14_valid),
        .in_map(conv14_out),
        .out_map(relu14_out),
        .out_valid(),
        .done(relu14_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L14), .IMG_SIZE(OUT_L14), .SHIFT(8)
    ) quant14 (
        .clk(clk), .rst(rst),
        .in_valid(quant14_valid),
        .in_map(relu14_out),
        .out_map(quant14_out),
        .out_valid(),
        .done(quant14_done)
    );

// Layer 15
       conv #(
        .CH_IN(CH_OUT_L14), .CH_OUT(CH_OUT_L15),
        .IMG_SIZE(IMG_L15), .K(K_L15),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L15), .STRIDE(STR_L15)
    ) conv15 (
        .clk(clk), .rst(rst),
        .in_valid(conv15_valid),
        .image(quant14_out),
        .filter(filter_l15),
        .bias(bias_l15),
        .op_img(conv15_out),
        .out_valid(),
        .done(conv15_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L15), .IMG_SIZE(OUT_L15), .DATA_W(PIXEL_ACC)
    ) relu15 (
        .clk(clk), .rst(rst),
        .in_valid(relu15_valid),
        .in_map(conv15_out),
        .out_map(relu15_out),
        .out_valid(),
        .done(relu15_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L15), .IMG_SIZE(OUT_L15), .SHIFT(8)
    ) quant15 (
        .clk(clk), .rst(rst),
        .in_valid(quant15_valid),
        .in_map(relu15_out),
        .out_map(quant15_out),
        .out_valid(),
        .done(quant15_done)
    );

    // Layer 16
       conv #(
        .CH_IN(CH_OUT_L15), .CH_OUT(CH_OUT_L16),
        .IMG_SIZE(IMG_L16), .K(K_L16),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L16), .STRIDE(STR_L16)
    ) conv16 (
        .clk(clk), .rst(rst),
        .in_valid(conv16_valid),
        .image(quant15_out),
        .filter(filter_l16),
        .bias(bias_l16),
        .op_img(conv16_out),
        .out_valid(),
        .done(conv16_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L16), .IMG_SIZE(OUT_L16), .DATA_W(PIXEL_ACC)
    ) relu16 (
        .clk(clk), .rst(rst),
        .in_valid(relu16_valid),
        .in_map(conv16_out),
        .out_map(relu16_out),
        .out_valid(),
        .done(relu16_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L16), .IMG_SIZE(OUT_L16), .SHIFT(8)
    ) quant16 (
        .clk(clk), .rst(rst),
        .in_valid(quant16_valid),
        .in_map(relu16_out),
        .out_map(quant16_out),
        .out_valid(),
        .done(quant16_done)
    );

    // Layer 19
       conv #(
        .CH_IN(CH_OUT_L16), .CH_OUT(CH_OUT_L19),
        .IMG_SIZE(IMG_L19), .K(K_L19),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L19), .STRIDE(STR_L19)
    ) conv19 (
        .clk(clk), .rst(rst),
        .in_valid(conv19_valid),
        .image(quant16_out),
        .filter(filter_l19),
        .bias(bias_l19),
        .op_img(conv19_out),
        .out_valid(),
        .done(conv19_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L19), .IMG_SIZE(OUT_L19), .DATA_W(PIXEL_ACC)
    ) relu19 (
        .clk(clk), .rst(rst),
        .in_valid(relu19_valid),
        .in_map(conv19_out),
        .out_map(relu19_out),
        .out_valid(),
        .done(relu19_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L19), .IMG_SIZE(OUT_L19), .SHIFT(8)
    ) quant19 (
        .clk(clk), .rst(rst),
        .in_valid(quant19_valid),
        .in_map(relu19_out),
        .out_map(quant19_out),
        .out_valid(),
        .done(quant19_done)
    );

upsample #(
    .CH(CH_OUT_L19) , .IN_SIZE(IMG_L20) , .FACTOR(FACTOR_L20)
) upsample20 (
    .clk(clk) , .rst(rst) ,
    .in_valid(upsample20_valid) , 
    .in_map(quant19_out) ,
    .out_map(upsample20_out) ,
    .out_valid() , 
    .done(upsample20_done)
);
// Layer 22
       conv #(
        .CH_IN(CH_UPL20), .CH_OUT(CH_OUT_L22),
        .IMG_SIZE(IMG_L22), .K(K_L22),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L22), .STRIDE(STR_L22)
    ) conv22 (
        .clk(clk), .rst(rst),
        .in_valid(conv22_valid),
        .image(upsample20_out),
        .filter(filter_l22),
        .bias(bias_l22),
        .op_img(conv22_out),
        .out_valid(),
        .done(conv22_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L22), .IMG_SIZE(OUT_L22), .DATA_W(PIXEL_ACC)
    ) relu22 (
        .clk(clk), .rst(rst),
        .in_valid(relu22_valid),
        .in_map(conv22_out),
        .out_map(relu22_out),
        .out_valid(),
        .done(relu22_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L22), .IMG_SIZE(OUT_L22), .SHIFT(8)
    ) quant22 (
        .clk(clk), .rst(rst),
        .in_valid(quant22_valid),
        .in_map(relu22_out),
        .out_map(quant22_out),
        .out_valid(),
        .done(quant22_done)
    );

// Layer 23
       conv #(
        .CH_IN(CH_OUT_L22), .CH_OUT(CH_OUT_L23),
        .IMG_SIZE(IMG_L23), .K(K_L23),
        .PIXEL_SIZE_IN(PIXEL_IN),
        .PIXEL_SIZE_OUT(PIXEL_ACC),
        .PADDING(PAD_L23), .STRIDE(STR_L23)
    ) conv23 (
        .clk(clk), .rst(rst),
        .in_valid(conv23_valid),
        .image(quant22_out),
        .filter(filter_l23),
        .bias(bias_l23),
        .op_img(conv23_out),
        .out_valid(),
        .done(conv23_done)
    );

    // ---------- ReLU ----------
    relu_map #(
        .CH(CH_OUT_L23), .IMG_SIZE(OUT_L23), .DATA_W(PIXEL_ACC)
    ) relu23 (
        .clk(clk), .rst(rst),
        .in_valid(relu23_valid),
        .in_map(conv23_out),
        .out_map(relu23_out),
        .out_valid(),
        .done(relu23_done)
    );

    // ---------- Quantization ----------
    quant_map #(
        .CH(CH_OUT_L23), .IMG_SIZE(OUT_L23), .SHIFT(8)
    ) quant23 (
        .clk(clk), .rst(rst),
        .in_valid(quant23_valid),
        .in_map(relu23_out),
        .out_map(quant23_out),
        .out_valid(),
        .done(quant23_done)
    );
endmodule
