library verilog;
use verilog.vl_types.all;
entity layer_scheduler is
    generic(
        NUM_LAYERS      : integer := 19;
        DATA_WIDTH      : integer := 8;
        MAX_CH          : integer := 256;
        MAX_DIM         : integer := 128
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        start           : in     vl_logic;
        done            : out    vl_logic;
        busy            : out    vl_logic;
        current_layer   : out    vl_logic_vector;
        in_channels     : out    vl_logic_vector(8 downto 0);
        out_channels    : out    vl_logic_vector(8 downto 0);
        kernel_size     : out    vl_logic_vector(1 downto 0);
        stride          : out    vl_logic;
        pad_en          : out    vl_logic;
        act_mode        : out    vl_logic_vector(1 downto 0);
        pool_en         : out    vl_logic;
        img_dim         : out    vl_logic_vector(7 downto 0);
        layer_type      : out    vl_logic;
        rq_multiplier   : out    vl_logic_vector(15 downto 0);
        rq_shift        : out    vl_logic_vector(5 downto 0);
        rq_zero_point   : out    vl_logic_vector;
        wt_load_start   : out    vl_logic;
        wt_load_done    : in     vl_logic;
        compute_start   : out    vl_logic;
        compute_done    : in     vl_logic;
        buf_swap        : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of NUM_LAYERS : constant is 1;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of MAX_CH : constant is 1;
    attribute mti_svvh_generic_type of MAX_DIM : constant is 1;
end layer_scheduler;
