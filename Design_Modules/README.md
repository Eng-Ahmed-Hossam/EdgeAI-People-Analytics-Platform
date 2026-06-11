# EdgeAI People Analytics Platform — Zynq CNN Accelerator

INT8 Tiny-YOLOv3 CNN accelerator for person detection/counting, implemented as a
**Zynq-7000 PS/PL co-design** and verified bit-exact against a Python golden model.

## Status

| Stage | Result |
|-------|--------|
| CNN core (19 layers, INT8) | ✅ Verified bit-exact vs Python golden |
| PL integration (weight/bias/reorder DMA, AXI-Lite regs) | ✅ Testbenched |
| Vivado 2018.2 build (xc7z100, full PS7 block design) | ✅ Synth + P&R + bitstream |
| Timing | ✅ Met @ 100 MHz |
| ARM software stack (decode + NMS + person count) | ✅ Builds (`edgeai_app.elf`) + PC-validated vs golden |

Post-route utilization: ~7.9k LUTs / 417 BRAM / 7 DSP on xc7z100.

## Layout

```
camera/     OV7670 DVP capture interface
frame/      pixel FIFO (CDC), frame writer / buffer manager
core/       CNN datapath — compute (PE array), post (bias/requant/act/pool), buffers, scheduler
pl/         AXI integration — weight_dma, bias_dma, reorder_dma, edgeai_ctrl_regs (+ testbenches)
top/        pl_edgeai_top (synthesis top) + tensor_output_buffer
include/    edgeai_defs.vh global defines
constraints/timing XDC
sw/         ARM C stack — edgeai_pl driver, yolo_decode, nms, person_count, main
sim/        Python golden models, ModelSim .do scripts, SystemVerilog testbenches, rtl_hex data
docs/       deployment readiness report
```

## Notes

- The quantized model at 128×128 / 4×4 grid is a coarse proof-of-pipeline; detection
  accuracy on full-scene photos is limited (the RTL faithfully reproduces the golden output).
- External reference model assets (`Golden_Model/`, model weights, videos) are excluded from
  this repo — it contains the FPGA co-design source and software only.
