library verilog;
use verilog.vl_types.all;
entity pixel_fifo is
    generic(
        DATA_WIDTH      : integer := 16;
        ADDR_WIDTH      : integer := 9
    );
    port(
        wr_clk          : in     vl_logic;
        wr_rst          : in     vl_logic;
        wr_en           : in     vl_logic;
        wr_data         : in     vl_logic_vector;
        full            : out    vl_logic;
        almost_full     : out    vl_logic;
        rd_clk          : in     vl_logic;
        rd_rst          : in     vl_logic;
        rd_en           : in     vl_logic;
        rd_data         : out    vl_logic_vector;
        empty           : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 1;
end pixel_fifo;
