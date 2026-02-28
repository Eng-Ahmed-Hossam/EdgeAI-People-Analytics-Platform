library verilog;
use verilog.vl_types.all;
entity requantizer is
    generic(
        ACC_WIDTH       : integer := 32;
        DATA_WIDTH      : integer := 8;
        MULT_WIDTH      : integer := 16;
        SHIFT_WIDTH     : integer := 6
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        data_in         : in     vl_logic_vector;
        in_valid        : in     vl_logic;
        multiplier      : in     vl_logic_vector;
        shift           : in     vl_logic_vector;
        zero_point      : in     vl_logic_vector;
        data_out        : out    vl_logic_vector;
        out_valid       : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of MULT_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of SHIFT_WIDTH : constant is 1;
end requantizer;
