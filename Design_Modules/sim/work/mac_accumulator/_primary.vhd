library verilog;
use verilog.vl_types.all;
entity mac_accumulator is
    generic(
        ACC_WIDTH       : integer := 32;
        MAC_WIDTH       : integer := 40
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        data_in         : in     vl_logic_vector;
        mode            : in     vl_logic_vector(1 downto 0);
        acc_out         : out    vl_logic_vector;
        acc_full        : out    vl_logic_vector;
        overflow        : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of MAC_WIDTH : constant is 1;
end mac_accumulator;
