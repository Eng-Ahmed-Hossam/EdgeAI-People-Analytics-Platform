library verilog;
use verilog.vl_types.all;
entity cnn_input_adapter is
    generic(
        IMG_W           : integer := 128;
        IMG_H           : integer := 128;
        DATA_WIDTH      : integer := 8;
        SDRAM_AW        : integer := 24;
        SDRAM_DW        : integer := 16;
        BUF_AW          : integer := 14
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        start           : in     vl_logic;
        frame_base_addr : in     vl_logic_vector;
        done            : out    vl_logic;
        sdram_rd_req    : out    vl_logic;
        sdram_rd_addr   : out    vl_logic_vector;
        sdram_rd_data   : in     vl_logic_vector;
        sdram_rd_valid  : in     vl_logic;
        ifmap_wr_en     : out    vl_logic;
        ifmap_wr_addr   : out    vl_logic_vector;
        ifmap_wr_data   : out    vl_logic_vector
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of IMG_W : constant is 1;
    attribute mti_svvh_generic_type of IMG_H : constant is 1;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of SDRAM_AW : constant is 1;
    attribute mti_svvh_generic_type of SDRAM_DW : constant is 1;
    attribute mti_svvh_generic_type of BUF_AW : constant is 1;
end cnn_input_adapter;
