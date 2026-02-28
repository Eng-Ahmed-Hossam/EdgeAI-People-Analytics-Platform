library verilog;
use verilog.vl_types.all;
entity sdram_ctrl is
    generic(
        ADDR_WIDTH      : integer := 24;
        DATA_WIDTH      : integer := 16;
        BURST_LEN       : integer := 8;
        CAS_LATENCY     : integer := 2;
        ROW_WIDTH       : integer := 13;
        COL_WIDTH       : integer := 9;
        BANK_WIDTH      : integer := 2
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        wr_req          : in     vl_logic;
        wr_addr         : in     vl_logic_vector;
        wr_data         : in     vl_logic_vector;
        wr_ack          : out    vl_logic;
        rd_req          : in     vl_logic;
        rd_addr         : in     vl_logic_vector;
        rd_data         : out    vl_logic_vector;
        rd_valid        : out    vl_logic;
        rd_ack          : out    vl_logic;
        sdram_clk       : out    vl_logic;
        sdram_cke       : out    vl_logic;
        sdram_cs_n      : out    vl_logic;
        sdram_ras_n     : out    vl_logic;
        sdram_cas_n     : out    vl_logic;
        sdram_we_n      : out    vl_logic;
        sdram_addr      : out    vl_logic_vector;
        sdram_ba        : out    vl_logic_vector;
        sdram_dq        : inout  vl_logic_vector;
        sdram_dqm       : out    vl_logic_vector(1 downto 0)
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BURST_LEN : constant is 1;
    attribute mti_svvh_generic_type of CAS_LATENCY : constant is 1;
    attribute mti_svvh_generic_type of ROW_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of COL_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BANK_WIDTH : constant is 1;
end sdram_ctrl;
