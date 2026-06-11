library verilog;
use verilog.vl_types.all;
entity cnn_core is
    generic(
        DATA_WIDTH      : integer := 8;
        ACC_WIDTH       : integer := 32;
        PE_ROWS         : integer := 16;
        PE_COLS         : integer := 16;
        MAX_K           : integer := 3;
        MAX_DIM         : integer := 128;
        MAX_CH          : integer := 256;
        BUF_DEPTH_FMAP  : vl_notype;
        BUF_AW_FMAP     : vl_notype;
        WB_DEPTH        : integer := 65536;
        WB_AW           : vl_notype
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        in_channels     : in     vl_logic_vector(8 downto 0);
        out_channels    : in     vl_logic_vector(8 downto 0);
        kernel_size     : in     vl_logic_vector(1 downto 0);
        stride          : in     vl_logic;
        pad_en          : in     vl_logic;
        act_mode        : in     vl_logic_vector(1 downto 0);
        pool_en         : in     vl_logic;
        img_dim         : in     vl_logic_vector;
        layer_type      : in     vl_logic;
        rq_multiplier   : in     vl_logic_vector(15 downto 0);
        rq_shift        : in     vl_logic_vector(5 downto 0);
        rq_zero_point   : in     vl_logic_vector;
        compute_start   : in     vl_logic;
        compute_done    : out    vl_logic;
        wt_ld_en        : in     vl_logic;
        wt_ld_addr      : in     vl_logic_vector;
        wt_ld_data      : in     vl_logic_vector;
        wt_swap         : in     vl_logic;
        ifmap_ext_wr_en : in     vl_logic;
        ifmap_ext_wr_addr: in     vl_logic_vector;
        ifmap_ext_wr_data: in     vl_logic_vector;
        out_data        : out    vl_logic_vector;
        out_valid       : out    vl_logic;
        out_last        : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of ACC_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of PE_ROWS : constant is 1;
    attribute mti_svvh_generic_type of PE_COLS : constant is 1;
    attribute mti_svvh_generic_type of MAX_K : constant is 1;
    attribute mti_svvh_generic_type of MAX_DIM : constant is 1;
    attribute mti_svvh_generic_type of MAX_CH : constant is 1;
    attribute mti_svvh_generic_type of BUF_DEPTH_FMAP : constant is 3;
    attribute mti_svvh_generic_type of BUF_AW_FMAP : constant is 3;
    attribute mti_svvh_generic_type of WB_DEPTH : constant is 1;
    attribute mti_svvh_generic_type of WB_AW : constant is 3;
end cnn_core;
