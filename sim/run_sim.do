# ==============================================================================
# ModelSim Simulation Script for EdgeAI PL Integration Test
# ==============================================================================
# Usage: vsim -do sim/run_sim.do

# Step 1: Create work library
vlib work
vmap work work

puts "# =========================================="
puts "# Step 1: Work library created"
puts "# =========================================="

# Step 2: Compile RTL (Verilog-2001) — order by dependency
# Include path for edgeai_defs.vh
set INC_DIR "+incdir+../include"

puts "# =========================================="
puts "# Step 2: Compiling RTL modules"
puts "# =========================================="

# Camera subsystem
vlog $INC_DIR ../camera/rtl/camera_if.v
vlog $INC_DIR ../camera/rtl/sccb_ctrl.v
vlog $INC_DIR ../camera/rtl/xclk_gen.v

# Frame subsystem
vlog $INC_DIR ../frame/rtl/pixel_fifo.v
vlog $INC_DIR ../frame/rtl/frame_writer.v
vlog $INC_DIR ../frame/rtl/frame_buffer_mgr.v
vlog $INC_DIR ../frame/rtl/sdram_ctrl.v

# Compute path
vlog $INC_DIR ../core/compute/rtl/mac_accumulator.v
vlog $INC_DIR ../core/compute/rtl/pe.v
vlog $INC_DIR ../core/compute/rtl/pe_array.v
vlog $INC_DIR ../core/compute/rtl/window_gen.v

# Post-processing
vlog $INC_DIR ../core/post/rtl/bias_add.v
vlog $INC_DIR ../core/post/rtl/requantizer.v
vlog $INC_DIR ../core/post/rtl/activation_unit.v
vlog $INC_DIR ../core/post/rtl/pooling_unit.v

# Buffers
vlog $INC_DIR ../core/buffers/rtl/ifmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/ofmap_buffer.v
vlog $INC_DIR ../core/buffers/rtl/weight_buffer.v

# Core integration
vlog $INC_DIR ../core/rtl/cnn_core.v
vlog $INC_DIR ../core/rtl/cnn_input_adapter.v
vlog $INC_DIR ../core/rtl/layer_scheduler.v

# Top level
vlog $INC_DIR ../top/rtl/tensor_output_buffer.v
vlog $INC_DIR ../top/rtl/pl_edgeai_top.v

puts "# =========================================="
puts "# RTL compile complete — 23 modules"
puts "# =========================================="

# Step 3: Compile SystemVerilog testbench
puts "# =========================================="
puts "# Step 3: Compiling testbench (SystemVerilog)"
puts "# =========================================="

vlog -sv $INC_DIR ../top/tb/pl_edgeai_top_tb.sv

puts "# =========================================="
puts "# Testbench compile complete"
puts "# =========================================="

# Step 4: Elaborate and launch simulation
puts "# =========================================="
puts "# Step 4: Launching simulation"
puts "# =========================================="

vsim -voptargs="+acc" work.pl_edgeai_top_tb

# Step 5: Add waveforms
puts "# =========================================="
puts "# Step 5: Adding waveforms"
puts "# =========================================="

add wave -r /*
puts "# Waveforms added successfully"

# Step 6: Run simulation
puts "# =========================================="
puts "# Step 6: Running simulation"
puts "# =========================================="

run -all

puts "# =========================================="
puts "# Simulation complete"
puts "# =========================================="
