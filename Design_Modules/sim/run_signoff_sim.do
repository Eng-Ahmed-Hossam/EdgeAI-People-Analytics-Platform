# ==============================================================================
# ModelSim Simulation Script — Final Sign-Off (tb_signoff.sv)
#   Runs all 13 conv layers with full numerical statistics + L18 capture.
#   Usage:  vsim -c -do run_signoff_sim.do
# ==============================================================================

vlib work
vmap work work

set INC_DIR "+incdir+../include"

puts "# Compiling RTL ..."
vlog $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog $INC_DIR ../core/compute/rtl/pe.v
vlog $INC_DIR ../core/compute/rtl/pe_array.v
vlog $INC_DIR ../core/compute/rtl/window_gen.v
vlog $INC_DIR ../core/post/rtl/bias_add.v
vlog $INC_DIR ../core/post/rtl/requantizer.v
vlog $INC_DIR ../core/post/rtl/activation_unit.v
vlog $INC_DIR ../core/post/rtl/pooling_unit.v
vlog $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/weight_buffer.v
vlog $INC_DIR ../core/rtl/cnn_core.v

puts "# Compiling testbench ..."
vlog -sv $INC_DIR tb_signoff.sv

puts "# Elaborating ..."
vsim -c -voptargs="+acc" work.tb_signoff

puts "# Running sign-off simulation ..."
run -all

puts "# Done."
quit
