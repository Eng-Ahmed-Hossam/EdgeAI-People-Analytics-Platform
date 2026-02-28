library verilog;
use verilog.vl_types.all;
entity window_gen is
    generic(
        DATA_WIDTH      : integer := 8;
        MAX_K           : integer := 3;
        MAX_IMG_DIM     : integer := 128
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        kernel_size     : in     vl_logic_vector;
        row_size        : in     vl_logic_vector;
        pad_en          : in     vl_logic;
        stride          : in     vl_logic_vector;
        pixel_in        : in     vl_logic_vector;
        pixel_valid     : in     vl_logic;
        frame_sync      : in     vl_logic;
        window_data     : out    vl_logic_vector;
        window_valid    : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of MAX_K : constant is 1;
    attribute mti_svvh_generic_type of MAX_IMG_DIM : constant is 1;
end window_gen;
