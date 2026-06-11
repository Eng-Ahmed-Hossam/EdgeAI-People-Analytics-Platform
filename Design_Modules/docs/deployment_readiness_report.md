# EdgeAI Location Intelligence — FPGA Deployment Readiness Report
**Date:** 2026-06-09  
**Platform:** Xilinx Zynq-7000 (ZC702 / ZedBoard)  
**Model:** Tiny-YOLOv3, 128×128 input, INT8 quantised  

---

## 1. CNN Core Status — VERIFIED ✓

| Item | Status |
|------|--------|
| All 19 layers simulated (chained) | ✓ Phase 8 + Phase 10 done |
| Conv layers L0–L18 bit-exact match | ✓ 0 mismatches golden vs RTL |
| Pool layers L1, L3, L5, L7, L9, L11 | ✓ All 5 verified |
| Stride-1 darknet pool (L11) | ✓ Verified |
| Requantizer multiply-shift-clamp | ✓ Bit-exact |
| Leaky ReLU (>>3) | ✓ Bit-exact |
| Last layer L18 linear (no activation) | ✓ Correct |
| PE array 16×16 utilisation | ✓ Correct tiling |
| Weight buffer (1 MB) | ✓ Sized correctly |
| ifmap buffer (262 KB) | ✓ Sized correctly |

**CNN core is frozen. No modifications permitted unless a proven bug is found.**

---

## 2. PL Integration Modules — IMPLEMENTED ✓

### 2.1 weight_dma.v
- AXI4 burst-read master, HP2 port
- Internal ROM: 19-layer byte counts and DDR offsets
- 4-byte beat → 4 individual wt_ld_en cycles (no byte loss)
- Pool layers: instant ST_DONE without AXI transaction
- Testbench: `tb_weight_dma.sv` ✓

### 2.2 bias_dma.v
- AXI4 burst-read master, HP2 port (arbitrated)
- 32-bit per beat = one INT32 bias value direct to bias_rom
- 19-layer count/offset ROM (total 8,632 bytes)
- Pool layers: instant done
- Testbench: `tb_bias_dma.sv` ✓

### 2.3 reorder_dma.v
- Two-phase: capture to 262 KB staging BRAM, then AGU scatter
- Tile-major → channel-major transform: `ch=tile*16+row; addr=ch*H*W+y*W+x`
- Pool mode: sequential copy (no scatter)
- Last-layer bypass: no capture or scatter for L18 tensor
- Testbench: `tb_reorder_dma.sv` ✓ (262,144-byte L0 test)

### 2.4 edgeai_ctrl_regs.v
- AXI4-Lite slave, 7 registers (0x00–0x18)
- W1S CTRL pulses: inference_start, soft_reset, tensor_clear
- RO STATUS: busy, tensor_ready, inference_done, cam_frame_error
- Byte-strobe support on all RW registers
- Testbench: `tb_edgeai_ctrl_regs.sv` ✓

### 2.5 cnn_core.v — Integration ports added
- `bias_ld_en`, `bias_ld_addr[7:0]`, `bias_ld_data[31:0]` added
- Registered write path into `bias_rom[]` only — compute logic untouched
- Constraint: all arithmetic paths frozen ✓

### 2.6 pl_edgeai_top.v — v2.0 complete
- SDRAM controller removed; replaced with AXI HP port bundles
- HP0: frame write (camera → DDR)
- HP1: frame read (DDR → cnn_core ifmap via cnn_input_adapter)
- HP2: weight + bias DMA (arbitrated, weight_dma priority)
- GP0: edgeai_ctrl_regs AXI-Lite slave
- `wt_load_done_hw = wt_dma_done & bias_dma_done`
- `is_last_layer = (sched_layer == 5'd18)`

---

## 3. ARM PS Software Stack — COMPLETE ✓

| File | Purpose | Status |
|------|---------|--------|
| `sw/edgeai_pl.h/.c` | PL register driver, inference_start, poll | ✓ |
| `sw/yolo_decode.h/.c` | Dequant, sigmoid, box decode, anchor scale | ✓ |
| `sw/nms.h/.c` | Greedy class-aware NMS | ✓ |
| `sw/person_count.h/.c` | Filter class_id==0 | ✓ |
| `sw/main.c` | Vitis bare-metal inference loop | ✓ |

---

## 4. Simulation Infrastructure — COMPLETE ✓

| File | Purpose | Status |
|------|---------|--------|
| `pl/tb/tb_weight_dma.sv` | AXI slave model, 5 layer tests | ✓ |
| `pl/tb/tb_bias_dma.sv` | AXI slave model, 5 layer tests | ✓ |
| `pl/tb/tb_reorder_dma.sv` | 262 KB scatter golden check | ✓ |
| `pl/tb/tb_edgeai_ctrl_regs.sv` | AXI-Lite R/W compliance, pulses | ✓ |
| `sim/run_pl_tb.do` | ModelSim script for all 4 testbenches | ✓ |
| `sim/run_golden_real_images.py` | Real image golden inference + NMS | ✓ |

---

## 5. Remaining Work Before FPGA Synthesis

### 5.1 Vivado Block Design (CRITICAL — not automated)

The following must be done manually in Vivado IP Integrator:

1. **Import RTL as IPs:**
   - Package `pl_edgeai_top` as a custom IP via Tools → Create and Package New IP
   - Add to block design

2. **Connect AXI HP Ports (PS7 → PL):**
   - S_AXI_HP0 ← frame_writer AXI master (HP0 write bundle)
   - S_AXI_HP1 ← frame_reader AXI master (HP1 read bundle)
   - S_AXI_HP2 ← weight_dma / bias_dma arbiter (HP2 read bundle)
   - M_AXI_GP0 → edgeai_ctrl_regs AXI-Lite slave

3. **Connect AXI HP wrappers:** The following IPs are referenced as flat wire bundles in `pl_edgeai_top.v` but need full AXI wrappers generated in Vivado:
   - `axi_hp_frame_writer`: write master for camera frames to DDR
   - `axi_hp_frame_reader`: read master for frames to cnn_input_adapter

4. **Clock and Reset:**
   - PL fabric clock: FCLK_CLK0 at 100 MHz → clk
   - Reset: FCLK_RESET0_N → !rst

5. **OV7670 camera interface:**
   - `cam_pclk`, `cam_href`, `cam_vsync`, `cam_data[7:0]` → GPIO/EMIO or direct PL pins
   - Pin assignment XDC for ZC702/ZedBoard header connectors

6. **Address editor:**
   - edgeai_ctrl_regs at 0x43C0_0000 (GP0 slave)
   - Weight blob DDR at 0x0021_0000
   - Bias blob DDR at 0x0044_0000
   - Frame buf A at 0x0020_0000, B at 0x0020_8000
   - Tensor DST at 0x0044_3000

### 5.2 AXI HP Wrapper RTL (required before synthesis)

`frame_writer.v` and `frame_reader.v` currently use internal behavioral SDRAM simulation. They need to be replaced with AXI4 masters that drive the HP0 / HP1 bundles exposed in `pl_edgeai_top.v`. Estimated effort: 2 modules, each ~150 lines.

### 5.3 Vitis BSP and Linker Script

- Platform created from exported Vivado hardware (`.xsa`)
- Linker script: place `.text` / `.bss` in DDR (0x0010_0000 base)
- Heap/stack: 64 KB each
- Add `xiltimer` library for poll timeout

### 5.4 Weight and Bias Blob Preparation

Weight and bias data must be pre-converted from per-layer hex files into a single flat binary blob and loaded into DDR at boot:

```python
# merge_weights.py (run once to generate boot assets)
import numpy as np, os
RTL_HEX = "sim/rtl_hex"
out = bytearray()
# Layer order: L0, L2, L4, L6, L8, L10, L12, L13, L14, L15, L16, L17, L18
conv_layers = [0,2,4,6,8,10,12,13,14,15,16,17,18]
for l in conv_layers:
    raw = bytes(int(ln,16) for ln in open(f"{RTL_HEX}/layer{l}_weights_flat.hex") if ln.strip())
    out += raw
open("weights.bin", "wb").write(out)  # copy to DDR at 0x0021_0000 via FSBL
```

Similarly for biases (INT32 little-endian).

### 5.5 OV7670 Initialisation

`main.c` references camera readiness via a `usleep` stub. Add SCCB (I2C) initialisation sequence to configure OV7670 for:
- RGB565 output mode
- QVGA (320×240) → PL crops and scales to 128×128

---

## 6. RTL Gaps Summary

| Gap | Severity | Notes |
|-----|----------|-------|
| `axi_hp_frame_writer.v` | HIGH | Needed for AXI HP0 camera frame DMA |
| `axi_hp_frame_reader.v` | HIGH | Needed for AXI HP1 ifmap DMA to cnn_core |
| HP2 mux back-pressure | MEDIUM | Current mux is combinatorial; HP2 AXI slave may need registered handshake |
| Tensor DMA to DDR | MEDIUM | `tensor_output_buffer` → DDR scatter not implemented; ARM polls from fixed DDR address |
| OV7670 SCCB init | LOW | Init sequence needed but camera default mode may work |

---

## 7. FPGA Deployment Checklist

- [ ] Run `run_golden_real_images.py` — verify person counts match ground truth
- [ ] Run `do run_pl_tb.do` in ModelSim — all 4 testbenches PASS
- [ ] Implement `axi_hp_frame_writer.v` and `axi_hp_frame_reader.v`
- [ ] Build Vivado block design, run implementation, check timing closure at 100 MHz
- [ ] Verify HP2 arbiter timing under simultaneous weight+bias DMA
- [ ] Generate `weights.bin` and `biases.bin` from rtl_hex files
- [ ] Create Vitis platform from exported .xsa
- [ ] Build `main.c` with edgeai_pl, yolo_decode, nms, person_count
- [ ] Program FPGA, preload DDR with weight/bias blobs via FSBL or U-Boot
- [ ] Run hardware inference on bus.jpg frame → verify person_count=4
- [ ] Run hardware inference on zidane.jpg frame → verify person_count=2
- [ ] Measure end-to-end latency (target: < 100 ms per frame at 100 MHz)

---

## 8. File Index

```
RTL/
├── core/rtl/
│   └── cnn_core.v             (bias_ld ports added — compute frozen)
├── pl/rtl/
│   ├── weight_dma.v
│   ├── bias_dma.v
│   ├── reorder_dma.v
│   └── edgeai_ctrl_regs.v
├── pl/tb/
│   ├── tb_weight_dma.sv
│   ├── tb_bias_dma.sv
│   ├── tb_reorder_dma.sv
│   └── tb_edgeai_ctrl_regs.sv
├── top/rtl/
│   └── pl_edgeai_top.v        (v2.0 — Zynq AXI HP)
├── sim/
│   ├── run_pl_tb.do
│   ├── golden_network.py
│   └── run_golden_real_images.py
├── sw/
│   ├── edgeai_pl.h/.c
│   ├── yolo_decode.h/.c
│   ├── nms.h/.c
│   ├── person_count.h/.c
│   └── main.c
└── docs/
    └── deployment_readiness_report.md   (this file)
```
