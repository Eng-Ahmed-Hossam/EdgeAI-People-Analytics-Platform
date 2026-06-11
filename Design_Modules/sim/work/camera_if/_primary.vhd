library verilog;
use verilog.vl_types.all;
entity camera_if is
    generic(
        IMG_W           : integer := 128;
        IMG_H           : integer := 128;
        PIXEL_WIDTH     : integer := 16
    );
    port(
        pclk            : in     vl_logic;
        rst             : in     vl_logic;
        vsync           : in     vl_logic;
        href            : in     vl_logic;
        d               : in     vl_logic_vector(7 downto 0);
        pixel_data      : out    vl_logic_vector;
        pixel_valid     : out    vl_logic;
        frame_start     : out    vl_logic;
        frame_end       : out    vl_logic;
        frame_error     : out    vl_logic
    );
    attribute mti_svvh_generic_type : integer;
    attribute mti_svvh_generic_type of IMG_W : constant is 1;
    attribute mti_svvh_generic_type of IMG_H : constant is 1;
    attribute mti_svvh_generic_type of PIXEL_WIDTH : constant is 1;
end camera_if;
