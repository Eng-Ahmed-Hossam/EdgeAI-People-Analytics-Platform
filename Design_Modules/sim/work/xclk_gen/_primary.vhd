library verilog;
use verilog.vl_types.all;
entity xclk_gen is
    generic(
        DIV_FACTOR      : integer := 2
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        xclk            : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DIV_FACTOR : constant is 1;
end xclk_gen;
