@echo off
REM ==============================================================================
REM EdgeAI PL Integration — ModelSim Batch Compile & Simulation
REM ==============================================================================
REM Usage: cd sim && compile_and_run.bat

set RTL=..
set INC=+incdir+%RTL%\include

echo.
echo # ==================================================================
echo # STEP 1: Creating ModelSim work library
echo # ==================================================================
if exist work rmdir /s /q work
vlib work
vmap work work

echo.
echo # ==================================================================
echo # STEP 2: Compiling RTL (Verilog-2001) — 23 modules
echo # ==================================================================

echo # [1/23] camera_if.v
vlog %INC% %RTL%\camera\rtl\camera_if.v
if errorlevel 1 goto :error

echo # [2/23] sccb_ctrl.v
vlog %INC% %RTL%\camera\rtl\sccb_ctrl.v
if errorlevel 1 goto :error

echo # [3/23] xclk_gen.v
vlog %INC% %RTL%\camera\rtl\xclk_gen.v
if errorlevel 1 goto :error

echo # [4/23] pixel_fifo.v
vlog %INC% %RTL%\frame\rtl\pixel_fifo.v
if errorlevel 1 goto :error

echo # [5/23] frame_writer.v
vlog %INC% %RTL%\frame\rtl\frame_writer.v
if errorlevel 1 goto :error

echo # [6/23] frame_buffer_mgr.v
vlog %INC% %RTL%\frame\rtl\frame_buffer_mgr.v
if errorlevel 1 goto :error

echo # [7/23] sdram_ctrl.v
vlog %INC% %RTL%\frame\rtl\sdram_ctrl.v
if errorlevel 1 goto :error

echo # [8/23] mac_accumulator.v
vlog %INC% %RTL%\core\compute\rtl\mac_accumulator.v
if errorlevel 1 goto :error

echo # [9/23] pe.v
vlog %INC% %RTL%\core\compute\rtl\pe.v
if errorlevel 1 goto :error

echo # [10/23] pe_array.v
vlog %INC% %RTL%\core\compute\rtl\pe_array.v
if errorlevel 1 goto :error

echo # [11/23] window_gen.v
vlog %INC% %RTL%\core\compute\rtl\window_gen.v
if errorlevel 1 goto :error

echo # [12/23] bias_add.v
vlog %INC% %RTL%\core\post\rtl\bias_add.v
if errorlevel 1 goto :error

echo # [13/23] requantizer.v
vlog %INC% %RTL%\core\post\rtl\requantizer.v
if errorlevel 1 goto :error

echo # [14/23] activation_unit.v
vlog %INC% %RTL%\core\post\rtl\activation_unit.v
if errorlevel 1 goto :error

echo # [15/23] pooling_unit.v
vlog %INC% %RTL%\core\post\rtl\pooling_unit.v
if errorlevel 1 goto :error

echo # [16/23] ifmap_buffer.v
vlog %INC% %RTL%\core\buffers\rtl\ifmap_buffer.v
if errorlevel 1 goto :error

echo # [17/23] ofmap_buffer.v
vlog %INC% %RTL%\core\buffers\rtl\ofmap_buffer.v
if errorlevel 1 goto :error

echo # [18/23] weight_buffer.v
vlog %INC% %RTL%\core\buffers\rtl\weight_buffer.v
if errorlevel 1 goto :error

echo # [19/23] cnn_core.v
vlog %INC% %RTL%\core\rtl\cnn_core.v
if errorlevel 1 goto :error

echo # [20/23] cnn_input_adapter.v
vlog %INC% %RTL%\core\rtl\cnn_input_adapter.v
if errorlevel 1 goto :error

echo # [21/23] layer_scheduler.v
vlog %INC% %RTL%\core\rtl\layer_scheduler.v
if errorlevel 1 goto :error

echo # [22/23] tensor_output_buffer.v
vlog %INC% %RTL%\top\rtl\tensor_output_buffer.v
if errorlevel 1 goto :error

echo # [23/23] pl_edgeai_top.v
vlog %INC% %RTL%\top\rtl\pl_edgeai_top.v
if errorlevel 1 goto :error

echo.
echo # ==================================================================
echo # RTL COMPILE: ALL 23 MODULES PASSED
echo # ==================================================================

echo.
echo # ==================================================================
echo # STEP 3: Compiling Testbench (SystemVerilog)
echo # ==================================================================

vlog -sv %INC% %RTL%\top\tb\pl_edgeai_top_tb.sv
if errorlevel 1 goto :error

echo.
echo # ==================================================================
echo # TESTBENCH COMPILE: PASSED
echo # ==================================================================

echo.
echo # ==================================================================
echo # STEP 4: Running Simulation
echo # ==================================================================

vsim -c -voptargs="+acc" work.pl_edgeai_top_tb -do "run -all; quit -f"

echo.
echo # ==================================================================
echo # SIMULATION COMPLETE
echo # ==================================================================
goto :end

:error
echo.
echo # ==================================================================
echo # COMPILE ERROR — See above for details
echo # ==================================================================
exit /b 1

:end
exit /b 0
