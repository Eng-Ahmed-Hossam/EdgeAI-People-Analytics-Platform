library verilog;
use verilog.vl_types.all;
entity weight_buffer is
    generic(
        DATA_WIDTH      : integer := 8;
        BUF_DEPTH       : integer := 65536;
        ADDR_WIDTH      : vl_notype
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        ld_en           : in     vl_logic;
        ld_addr         : in     vl_logic_vector;
        ld_data         : in     vl_logic_vector;
        rd_en           : in     vl_logic;
        rd_addr         : in     vl_logic_vector;
        rd_data         : out    vl_logic_vector;
        rd_valid        : out    vl_logic;
        swap            : in     vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BUF_DEPTH : constant is 1;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 3;
end weight_buffer;
