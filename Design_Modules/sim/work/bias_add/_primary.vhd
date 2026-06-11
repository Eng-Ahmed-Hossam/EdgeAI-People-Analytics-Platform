library verilog;
use verilog.vl_types.all;
entity bias_add is
    generic(
        ACC_WIDTH       : integer := 32;
        BIAS_WIDTH      : integer := 32
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        acc_in          : in     vl_logic_vector;
        bias_in         : in     vl_logic_vector;
        in_valid        : in     vl_logic;
        result          : out    vl_logic_vector;
        out_valid       : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BIAS_WIDTH : constant is 1;
end bias_add;
