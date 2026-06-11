# ==============================================================================
# ModelSim Simulation Script — 5 Stride-2 Pool Layers (tb_pool_layers.sv)
# ==============================================================================
# Usage (from D:\Graduation_Project\RTL\sim\):
#   vsim -c -do run_pool_sim.do
# ==============================================================================

vlib pool_work
vmap pool_work pool_work

set INC_DIR "+incdir+../include"

puts "# Compiling RTL ..."
vlog -work pool_work $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog -work pool_work $INC_DIR ../core/compute/rtl/pe.v
vlog -work pool_work $INC_DIR ../core/compute/rtl/pe_array.v
vlog -work pool_work $INC_DIR ../core/compute/rtl/window_gen.v
vlog -work pool_work $INC_DIR ../core/post/rtl/bias_add.v
vlog -work pool_work $INC_DIR ../core/post/rtl/requantizer.v
vlog -work pool_work $INC_DIR ../core/post/rtl/activation_unit.v
vlog -work pool_work $INC_DIR ../core/post/rtl/pooling_unit.v
vlog -work pool_work $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog -work pool_work $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog -work pool_work $INC_DIR ../core/buffers/rtl/weight_buffer.v
vlog -work pool_work $INC_DIR ../core/rtl/cnn_core.v

puts "# Compiling testbench ..."
vlog -work pool_work -sv $INC_DIR tb_pool_layers.sv

puts "# Elaborating ..."
vsim -lib pool_work -c -voptargs="+acc" pool_work.tb_pool_layers

puts "# Running pool simulation (L1, L3, L5, L7, L9) ..."
run -all

puts "# Done."
quit
