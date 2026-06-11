vlib work
vmap work work
set INC_DIR "+incdir+../include"
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
vlog -sv $INC_DIR tb_signoff.sv
vsim -c -voptargs="+acc" work.tb_signoff
run -all
quit
