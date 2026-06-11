# ==============================================================================
# ModelSim Simulation Script — Layer 0 Verification (tb_layer0.sv)
# ==============================================================================
# Usage (from D:\Graduation_Project\RTL\sim\):
#   vsim -c -do run_layer0_sim.do
# Or from any directory:
#   vsim -c -do sim/run_layer0_sim.do
# ==============================================================================

# Step 1: Create work library
vlib work
vmap work work

puts "# =========================================="
puts "# Step 1: Work library created"
puts "# =========================================="

# Include path
set INC_DIR "+incdir+../include"

puts "# =========================================="
puts "# Step 2: Compiling RTL (dependency order)"
puts "# =========================================="

# PE compute path
vlog $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog $INC_DIR ../core/compute/rtl/pe.v
vlog $INC_DIR ../core/compute/rtl/pe_array.v
vlog $INC_DIR ../core/compute/rtl/window_gen.v

# Post-processing pipeline
vlog $INC_DIR ../core/post/rtl/bias_add.v
vlog $INC_DIR ../core/post/rtl/requantizer.v
vlog $INC_DIR ../core/post/rtl/activation_unit.v
vlog $INC_DIR ../core/post/rtl/pooling_unit.v

# Buffers
vlog $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/weight_buffer.v

# Core integration (cnn_core only — no top-level needed)
vlog $INC_DIR ../core/rtl/cnn_core.v

puts "# =========================================="
puts "# Step 3: Compiling SystemVerilog testbench"
puts "# =========================================="

vlog -sv $INC_DIR tb_layer0.sv

puts "# =========================================="
puts "# Step 4: Elaborating and launching simulation"
puts "# =========================================="

# -c = batch/console mode (no GUI)
# -voptargs=+acc = preserve hierarchy for $readmemh hierarchical access
vsim -c -voptargs="+acc" work.tb_layer0

puts "# =========================================="
puts "# Step 5: Running simulation"
puts "# (Expected ~8.8M cycles, timeout = 15M)"
puts "# =========================================="

run -all

puts "# =========================================="
puts "# Simulation complete"
puts "# =========================================="

quit
