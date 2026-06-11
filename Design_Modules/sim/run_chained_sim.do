# ==============================================================================
# ModelSim — TRUE end-to-end chained execution (tb_chained.sv)
#   Uses chain_work library to avoid clashing with the sign-off run on `work`.
#   Usage:  vsim -c -do run_chained_sim.do
# ==============================================================================

vlib chain_work
vmap chain_work chain_work

set INC_DIR "+incdir+../include"

puts "# Compiling RTL ..."
vlog -work chain_work $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog -work chain_work $INC_DIR ../core/compute/rtl/pe.v
vlog -work chain_work $INC_DIR ../core/compute/rtl/pe_array.v
vlog -work chain_work $INC_DIR ../core/compute/rtl/window_gen.v
vlog -work chain_work $INC_DIR ../core/post/rtl/bias_add.v
vlog -work chain_work $INC_DIR ../core/post/rtl/requantizer.v
vlog -work chain_work $INC_DIR ../core/post/rtl/activation_unit.v
vlog -work chain_work $INC_DIR ../core/post/rtl/pooling_unit.v
vlog -work chain_work $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog -work chain_work $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog -work chain_work $INC_DIR ../core/buffers/rtl/weight_buffer.v
vlog -work chain_work $INC_DIR ../core/rtl/cnn_core.v

puts "# Compiling testbench ..."
vlog -work chain_work -sv $INC_DIR tb_chained.sv

puts "# Elaborating ..."
vsim -lib chain_work -c -voptargs="+acc" chain_work.tb_chained

puts "# Running chained simulation ..."
run -all

puts "# Done."
quit
