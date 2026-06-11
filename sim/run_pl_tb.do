# =============================================================================
# run_pl_tb.do — ModelSim simulation script for PL integration testbenches
# Runs: tb_weight_dma, tb_bias_dma, tb_reorder_dma, tb_edgeai_ctrl_regs
#
# Usage (from ModelSim transcript):
#   do D:/Graduation_Project/RTL/sim/run_pl_tb.do
# =============================================================================

# Project root
set ROOT       "D:/Graduation_Project/RTL"
set LIB_DIR    "$ROOT/sim/work_pl"
set CORE_DIR   "$ROOT/core/rtl"
set PL_DIR     "$ROOT/pl/rtl"
set PL_TB      "$ROOT/pl/tb"

# --------------------------------------------------------------------------
# Create isolated library (avoids contaminating cnn_core library)
# --------------------------------------------------------------------------
if {[file exists $LIB_DIR]} {
    vdel -lib $LIB_DIR -all
}
vlib $LIB_DIR
vmap work $LIB_DIR

# --------------------------------------------------------------------------
# Compile RTL under test
# --------------------------------------------------------------------------
vlog -sv -work $LIB_DIR \
    +define+SIM \
    $PL_DIR/weight_dma.v

vlog -sv -work $LIB_DIR \
    +define+SIM \
    $PL_DIR/bias_dma.v

vlog -sv -work $LIB_DIR \
    +define+SIM \
    $PL_DIR/reorder_dma.v

vlog -sv -work $LIB_DIR \
    +define+SIM \
    $PL_DIR/edgeai_ctrl_regs.v

# --------------------------------------------------------------------------
# Compile testbenches
# --------------------------------------------------------------------------
vlog -sv -work $LIB_DIR $PL_TB/tb_weight_dma.sv
vlog -sv -work $LIB_DIR $PL_TB/tb_bias_dma.sv
vlog -sv -work $LIB_DIR $PL_TB/tb_reorder_dma.sv
vlog -sv -work $LIB_DIR $PL_TB/tb_edgeai_ctrl_regs.sv

# --------------------------------------------------------------------------
# Simulation helper proc
# --------------------------------------------------------------------------
proc run_tb {top timeout_us} {
    puts "\n============================================================"
    puts "  Running: $top"
    puts "============================================================"
    vsim -lib $::LIB_DIR -t 1ps \
        -novopt \
        $top
    run ${timeout_us}us
    quit -sim
}

# --------------------------------------------------------------------------
# Run all testbenches
# --------------------------------------------------------------------------
run_tb tb_weight_dma     5000
run_tb tb_bias_dma       2000
run_tb tb_reorder_dma    120000
run_tb tb_edgeai_ctrl_regs 500

puts "\n============================================================"
puts "  All PL integration testbenches complete."
puts "  Check 'PASS / FAIL' lines in transcript above."
puts "============================================================"
