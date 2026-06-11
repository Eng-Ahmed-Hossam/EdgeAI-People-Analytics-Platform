library verilog;
use verilog.vl_types.all;
entity pooling_unit is
    generic(
        DATA_WIDTH      : integer := 8;
        MAX_IMG_DIM     : integer := 128
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        row_size        : in     vl_logic_vector;
        bypass          : in     vl_logic;
        frame_sync      : in     vl_logic;
        data_in         : in     vl_logic_vector;
        in_valid        : in     vl_logic;
        data_out        : out    vl_logic_vector;
        out_valid       : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of MAX_IMG_DIM : constant is 1;
end pooling_unit;
