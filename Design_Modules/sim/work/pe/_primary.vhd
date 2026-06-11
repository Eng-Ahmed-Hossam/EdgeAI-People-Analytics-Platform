library verilog;
use verilog.vl_types.all;
entity pe is
    generic(
        DATA_WIDTH      : integer := 8;
        ACC_WIDTH       : integer := 32
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        en              : in     vl_logic;
        acc_clear       : in     vl_logic;
        ifmap_data      : in     vl_logic_vector;
        weight_data     : in     vl_logic_vector;
        acc_out         : out    vl_logic_vector
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
end pe;
