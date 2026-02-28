library verilog;
use verilog.vl_types.all;
entity pl_edgeai_top is
    generic(
        DATA_WIDTH      : integer := 8;
        ACC_WIDTH       : integer := 32;
        PE_ROWS         : integer := 16;
        PE_COLS         : integer := 16;
        SDRAM_AW        : integer := 24;
        SDRAM_DW        : integer := 16
    );
    port(
        sys_clk         : in     vl_logic;
        sys_rst         : in     vl_logic;
        cam_pclk        : in     vl_logic;
        cam_vsync       : in     vl_logic;
        cam_href        : in     vl_logic;
        cam_d           : in     vl_logic_vector(7 downto 0);
        cam_xclk        : out    vl_logic;
        sccb_scl        : out    vl_logic;
        sccb_sda        : inout  vl_logic;
        sdram_clk       : out    vl_logic;
        sdram_cke       : out    vl_logic;
        sdram_cs_n      : out    vl_logic;
        sdram_ras_n     : out    vl_logic;
        sdram_cas_n     : out    vl_logic;
        sdram_we_n      : out    vl_logic;
        sdram_addr      : out    vl_logic_vector(12 downto 0);
        sdram_ba        : out    vl_logic_vector(1 downto 0);
        sdram_dq        : inout  vl_logic_vector(15 downto 0);
        sdram_dqm       : out    vl_logic_vector(1 downto 0);
        inference_start : in     vl_logic;
        inference_done  : out    vl_logic;
        tensor_ready    : out    vl_logic;
        tensor_rd_en    : in     vl_logic;
        tensor_rd_addr  : in     vl_logic_vector(15 downto 0);
        tensor_rd_data  : out    vl_logic_vector;
        tensor_rd_valid : out    vl_logic;
        cam_frame_error : out    vl_logic;
        busy            : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of PE_ROWS : constant is 1;
    attribute mti_svvh_generic_type of PE_COLS : constant is 1;
    attribute mti_svvh_generic_type of SDRAM_AW : constant is 1;
    attribute mti_svvh_generic_type of SDRAM_DW : constant is 1;
end pl_edgeai_top;
