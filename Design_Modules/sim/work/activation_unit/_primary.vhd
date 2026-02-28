library verilog;
use verilog.vl_types.all;
entity activation_unit is
    generic(
        DATA_WIDTH      : integer := 8;
        LEAK_SHIFT      : integer := 3
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        data_in         : in     vl_logic_vector;
        in_valid        : in     vl_logic;
        mode            : in     vl_logic_vector(1 downto 0);
        data_out        : out    vl_logic_vector;
        out_valid       : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of LEAK_SHIFT : constant is 1;
end activation_unit;
