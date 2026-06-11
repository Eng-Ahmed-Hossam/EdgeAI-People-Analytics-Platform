library verilog;
use verilog.vl_types.all;
entity frame_buffer_mgr is
    generic(
        ADDR_WIDTH      : integer := 24;
        BUF_A_BASE      : vl_logic_vector(0 to 23) := (Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0);
        BUF_B_BASE      : vl_logic_vector(0 to 23) := (Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi1, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0, Hi0)
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        wr_done         : in     vl_logic;
        wr_base_addr    : out    vl_logic_vector;
        rd_done         : in     vl_logic;
        rd_base_addr    : out    vl_logic_vector;
        frame_ready     : out    vl_logic;
        buf_sel         : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BUF_A_BASE : constant is 1;
    attribute mti_svvh_generic_type of BUF_B_BASE : constant is 1;
end frame_buffer_mgr;
