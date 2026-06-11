# ==============================================================================
# ModelSim Simulation Script — All 13 Conv Layers (tb_conv_layers.sv)
# ==============================================================================
# Usage (from D:\Graduation_Project\RTL\sim\):
#   vsim -c -do run_conv_sim.do
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
vlog -sv $INC_DIR tb_conv_layers.sv

puts "# Elaborating ..."
vsim -c -voptargs="+acc" work.tb_conv_layers

puts "# Running simulation (all 13 conv layers) ..."
run -all

puts "# Done."
quit
