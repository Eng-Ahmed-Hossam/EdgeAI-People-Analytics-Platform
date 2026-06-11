library verilog;
use verilog.vl_types.all;
entity frame_writer is
    generic(
        IMG_W           : integer := 128;
        IMG_H           : integer := 128;
        PIXEL_WIDTH     : integer := 16;
        ADDR_WIDTH      : integer := 24;
        DATA_WIDTH      : integer := 16;
        BURST_LEN       : integer := 8
    );
    port(
        clk             : in     vl_logic;
        rst             : in     vl_logic;
        frame_start     : in     vl_logic;
        base_addr       : in     vl_logic_vector;
        write_done      : out    vl_logic;
        fifo_rd_en      : out    vl_logic;
        fifo_data       : in     vl_logic_vector;
        fifo_empty      : in     vl_logic;
        sdram_wr_req    : out    vl_logic;
        sdram_wr_addr   : out    vl_logic_vector;
        sdram_wr_data   : out    vl_logic_vector;
        sdram_wr_ack    : in     vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of IMG_W : constant is 1;
    attribute mti_svvh_generic_type of IMG_H : constant is 1;
    attribute mti_svvh_generic_type of PIXEL_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of ADDR_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of DATA_WIDTH : constant is 1;
    attribute mti_svvh_generic_type of BURST_LEN : constant is 1;
end frame_writer;
