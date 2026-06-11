library verilog;
use verilog.vl_types.all;
entity sccb_ctrl is
    generic(
        CLK_DIV         : integer := 250;
        DEVICE_ID       : vl_logic_vector(0 to 7) := (Hi0, Hi1, Hi0, Hi0, Hi0, Hi0, Hi1, Hi0)
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        start           : in     vl_logic;
        addr            : in     vl_logic_vector(7 downto 0);
        data_in         : in     vl_logic_vector(7 downto 0);
        done            : out    vl_logic;
        busy            : out    vl_logic;
        sccb_scl        : out    vl_logic;
        sccb_sda        : inout  vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of CLK_DIV : constant is 1;
    attribute mti_svvh_generic_type of DEVICE_ID : constant is 1;
end sccb_ctrl;
