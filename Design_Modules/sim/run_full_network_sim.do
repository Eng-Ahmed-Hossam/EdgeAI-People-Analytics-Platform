# ==============================================================================
# ModelSim Simulation Script — Phase 11: Full 19-Layer Network
# ==============================================================================
# Usage (from D:\Graduation_Project\RTL\sim\):
#   vsim -c -do run_full_network_sim.do
# ==============================================================================

vlib full_work
vmap full_work full_work

set INC_DIR "+incdir+../include"

puts "# Compiling RTL ..."

vlog -work full_work $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog -work full_work $INC_DIR ../core/compute/rtl/pe.v
vlog -work full_work $INC_DIR ../core/compute/rtl/pe_array.v
vlog -work full_work $INC_DIR ../core/compute/rtl/window_gen.v
vlog -work full_work $INC_DIR ../core/post/rtl/bias_add.v
vlog -work full_work $INC_DIR ../core/post/rtl/requantizer.v
vlog -work full_work $INC_DIR ../core/post/rtl/activation_unit.v
vlog -work full_work $INC_DIR ../core/post/rtl/pooling_unit.v
vlog -work full_work $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog -work full_work $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog -work full_work $INC_DIR ../core/buffers/rtl/weight_buffer.v
vlog -work full_work $INC_DIR ../core/rtl/cnn_core.v

puts "# Compiling testbench ..."
vlog -work full_work -sv $INC_DIR tb_full_network.sv

puts "# Elaborating ..."
vsim -lib full_work -c -voptargs="+acc" full_work.tb_full_network

puts "# Running simulation (18 layers: 13 conv + 5 pool, L11 skipped) ..."
run -all

puts "# Done."
quit
