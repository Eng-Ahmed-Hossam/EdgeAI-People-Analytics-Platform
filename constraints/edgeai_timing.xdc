# ============================================================
# EdgeAI Timing Constraints
# Target: xcvu13p-fsga2577-1-i
# Vivado 2018.2
# ============================================================
#
# IMPORTANT:
#
#   This project currently targets a Virtex UltraScale+
#   FPGA device:
#
#       xcvu13p-fsga2577-1-i
#
#   Previous revisions of this project may have originated
#   from a Zynq-based architecture. Therefore references to
#   PS7-generated clocks or Zynq-specific clocking should be
#   reviewed if the design is migrated between device families.
#
# ------------------------------------------------------------
# Clock Verification
# ------------------------------------------------------------
#
# After synthesis:
#
#       open_run synth_2
#       get_clocks
#
# Verified clocks:
#
#       cam_pclk
#       sys_clk
#
# Therefore the top-level clock ports are constrained
# explicitly below.
#
# ============================================================


# ============================================================
# External Camera Pixel Clock
# ============================================================
#
# Source:
#   OV7670 camera PCLK
#
# Top-level port:
#   cam_pclk
#
# Frequency:
#   24 MHz
#
# Period:
#   41.667 ns
#
create_clock \
    -name cam_pclk \
    -period 41.667 \
    [get_ports cam_pclk]


# ============================================================
# System Clock
# ============================================================
#
# Top-level port:
#   sys_clk
#
# Frequency:
#   100 MHz
#
# Period:
#   10.000 ns
#
# Verified from synthesis:
#
#   report_property [get_ports sys_clk]
#
#       DIRECTION   = IN
#       UNCONNECTED = 0
#
# Therefore sys_clk is an active top-level input clock and
# should be constrained explicitly.
#
create_clock \
    -name sys_clk \
    -period 10.000 \
    [get_ports sys_clk]


# ============================================================
# Camera XCLK
# ============================================================
#
# Verified from synthesis:
#
#   report_property [get_ports cam_xclk]
#
#       DIRECTION   = IN
#       UNCONNECTED = 1
#
# cam_xclk is currently unused by the synthesized design.
#
# No timing constraint is required.
#
# If future revisions of the design use cam_xclk as an
# actual clock source, add an appropriate create_clock
# constraint.
#
# ============================================================


# ============================================================
# Clock Domain Crossing (CDC)
# ============================================================
#
# Timing analysis identified crossings between:
#
#       sys_clk  (100 MHz)
#       cam_pclk (24 MHz)
#
# through asynchronous FIFO synchronizer logic:
#
#       rd_ptr_gray_sync*
#       wr_ptr_gray_sync*
#
# These signals are Gray-coded FIFO pointers crossing between
# independent clock domains through synchronizer stages.
#
# Without the following constraint Vivado attempts to perform
# setup and hold timing analysis across unrelated clocks,
# producing false CDC violations.
#
# The following constraint preserves timing analysis within
# each clock domain while excluding timing analysis between
# the asynchronous domains.
#
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks cam_pclk]


# ============================================================
# Implementation Notes
# ============================================================
#
# After modifying this file:
#
#   reset_run synth_2
#   launch_runs synth_2 -jobs 8
#   wait_on_run synth_2
#
# Then run implementation:
#
#   launch_runs impl_1 -jobs 8
#   wait_on_run impl_1
#
# Open the implemented design:
#
#   open_run impl_1
#
# Check timing:
#
#   report_timing_summary
#
# Post-synthesis hold violations should not be used as the
# final timing signoff metric. Evaluate hold timing after
# placement and routing is complete.
#
# ============================================================