library verilog;
use verilog.vl_types.all;
entity tensor_output_buffer is
    generic(
        DATA_WIDTH      : integer := 8;
        TENSOR_DEPTH    : integer := 4080;
        ADDR_WIDTH      : vl_notype
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        wr_en           : in     vl_logic;
        wr_data         : in     vl_logic_vector;
        tensor_clear    : in     vl_logic;
        rd_en           : in     vl_logic;
        rd_addr         : in     vl_logic_vector;
        rd_data         : out    vl_logic_vector;
        rd_valid        : out    vl_logic;
        tensor_ready    : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of TENSOR_DEPTH : constant is 1;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 3;
end tensor_output_buffer;
