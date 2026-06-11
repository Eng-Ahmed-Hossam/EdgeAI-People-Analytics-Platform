#!/usr/bin/env python3
"""
golden_layer0.py

Generate the Layer 0 (Conv 3->16, 3x3, pad=1, stride=1, LeakyReLU) golden package.

Bit-accurate pipeline matching RTL modules:
  acc (INT32) <- INT8*INT8 MAC sum (pe.v)
  acc += INT32 bias               (bias_add.v)
  out = clamp((acc * M) >> sh + zp, -128, 127)  (requantizer.v)
  out = out >= 0 ? out : out >> 3  (activation_unit.v, LEAK_SHIFT=3)

Outputs (written to rtl_hex/):
  layer0_input.hex           -- INT8 test image, layout: [ch][row][col], 1 byte/line
  layer0_expected_output.hex -- INT8 L0 output, layout: [oc][row][col], 1 byte/line
"""

import os
import json
import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_HEX    = os.path.join(SCRIPT_DIR, "rtl_hex")

# ── Network parameters (L0) ───────────────────────────────────────────────────
IMG_H  = 128
IMG_W  = 128
IN_CH  = 3
OUT_CH = 16
KH = KW = 3
PAD    = 1
STRIDE = 1
LEAK_SHIFT = 3

# ── Load quantizer params ─────────────────────────────────────────────────────
with open(os.path.join(RTL_HEX, "layer0_meta.json")) as f:
    meta = json.load(f)
RQ_M  = int(meta["requantizer"]["multiplier"])
RQ_SH = int(meta["requantizer"]["shift"])
RQ_ZP = int(meta["requantizer"]["zero_point"])
print(f"[L0] Requantizer: M={RQ_M}, shift={RQ_SH}, zp={RQ_ZP}")

# ── Load weights [OUT_CH, IN_CH, KH, KW] from flat hex (1 byte/line) ─────────
wt_path = os.path.join(RTL_HEX, "layer0_weights_flat.hex")
with open(wt_path) as f:
    raw = np.array([int(ln.strip(), 16) for ln in f if ln.strip()], dtype=np.uint8)
weights = raw.view(np.int8).reshape(OUT_CH, IN_CH, KH, KW)
print(f"[L0] Weights: shape={weights.shape}  range=[{weights.min()}, {weights.max()}]")
assert weights.shape == (OUT_CH, IN_CH, KH, KW), "Weight shape mismatch"

# ── Load biases [OUT_CH] INT32 (8 hex chars / line, two's-complement) ─────────
bias_path = os.path.join(RTL_HEX, "layer0_bias.hex")
with open(bias_path) as f:
    raw_bias = np.array([int(ln.strip(), 16) for ln in f if ln.strip()], dtype=np.uint32)
biases = raw_bias.view(np.int32)
print(f"[L0] Biases:  count={len(biases)}  range=[{biases.min()}, {biases.max()}]")
assert len(biases) == OUT_CH, f"Expected {OUT_CH} biases, got {len(biases)}"

# ── Generate deterministic INT8 test input ────────────────────────────────────
# Pattern: inp[c,y,x] = (c*43 + y*7 + x*3) mod 256, mapped to [-128,127]
ys = np.arange(IMG_H, dtype=np.int32)
xs = np.arange(IMG_W, dtype=np.int32)
YY, XX = np.meshgrid(ys, xs, indexing='ij')  # [128, 128]

inp = np.zeros((IN_CH, IMG_H, IMG_W), dtype=np.int8)
for c in range(IN_CH):
    raw_c = (int(c) * 43 + YY * 7 + XX * 3) % 256
    inp[c] = (raw_c - 128).astype(np.int8)

print(f"[L0] Input:  shape={inp.shape}  range=[{inp.min()}, {inp.max()}]")

# ── Zero-pad input ────────────────────────────────────────────────────────────
inp_padded = np.zeros((IN_CH, IMG_H + 2*PAD, IMG_W + 2*PAD), dtype=np.int64)
inp_padded[:, PAD:PAD+IMG_H, PAD:PAD+IMG_W] = inp.astype(np.int64)

out_h = IMG_H  # same-size output (pad=1 for 3x3, stride=1)
out_w = IMG_W

# ── Convolution (INT64 accumulator, bit-exact) ────────────────────────────────
# acc[oc, y, x] = sum_{ic,ky,kx} inp[ic, y+ky, x+kx] * w[oc, ic, ky, kx]
acc = np.zeros((OUT_CH, out_h, out_w), dtype=np.int64)

for ic in range(IN_CH):
    for ky in range(KH):
        for kx in range(KW):
            patch = inp_padded[ic, ky:ky+out_h, kx:kx+out_w]  # [128, 128], int64
            for oc in range(OUT_CH):
                acc[oc] += patch * int(weights[oc, ic, ky, kx])

# ── Add biases ────────────────────────────────────────────────────────────────
acc += biases.astype(np.int64).reshape(OUT_CH, 1, 1)
print(f"[L0] Acc+bias range: [{acc.min()}, {acc.max()}]")

# ── Requantize: product = acc * M, then arithmetic-shift by shift ─────────────
# M is 16-bit unsigned, treated as positive 17-bit signed in RTL ({1'b0, multiplier})
product = acc * np.int64(RQ_M)        # [OUT_CH, 128, 128], int64
# Arithmetic right shift. Python/numpy int64 >> is arithmetic for signed types.
shifted = product >> np.int64(RQ_SH)
with_zp  = shifted + np.int64(RQ_ZP)
clamped  = np.clip(with_zp, -128, 127).astype(np.int8)
print(f"[L0] After requant range: [{clamped.min()}, {clamped.max()}]")

# ── LeakyReLU: negative values >> LEAK_SHIFT (arithmetic, INT8 context) ───────
# Use int16 intermediate to avoid any int8 shift-direction ambiguity in NumPy
c16  = clamped.astype(np.int16)
output = np.where(c16 >= 0, c16, c16 >> LEAK_SHIFT).astype(np.int8)
print(f"[L0] After LeakyReLU range: [{output.min()}, {output.max()}]")

nonzero = np.count_nonzero(output)
print(f"[L0] Non-zero outputs: {nonzero} / {output.size} ({100*nonzero/output.size:.1f}%)")

# ── Write input hex (1 byte/line, 2 hex chars, unsigned representation) ───────
input_hex = os.path.join(RTL_HEX, "layer0_input.hex")
with open(input_hex, "w") as f:
    for c in range(IN_CH):
        for y in range(IMG_H):
            for x in range(IMG_W):
                f.write("{:02x}\n".format(int(inp[c, y, x]) & 0xFF))
print(f"[L0] Input  written -> {input_hex}  ({IN_CH*IMG_H*IMG_W} lines)")

# ── Write expected output hex (1 byte/line, 2 hex chars) — [oc][y][x] order ──
out_hex = os.path.join(RTL_HEX, "layer0_expected_output.hex")
with open(out_hex, "w") as f:
    for oc in range(OUT_CH):
        for y in range(out_h):
            for x in range(out_w):
                f.write("{:02x}\n".format(int(output[oc, y, x]) & 0xFF))
print(f"[L0] Output written -> {out_hex}  ({OUT_CH*out_h*out_w} lines)")

# ── Write RTL-order output hex — [oy][ox][oc] order (matches new FSM stream) ──
# RTL FSM loop: oc_tile -> oy -> ox -> post_feed(row=0..15)
# For L0 (single tile): output order is [oy][ox][oc=0..15]
rtl_hex_path = os.path.join(RTL_HEX, "layer0_expected_rtl_order.hex")
with open(rtl_hex_path, "w") as f:
    for y in range(out_h):
        for x in range(out_w):
            for oc in range(OUT_CH):
                f.write("{:02x}\n".format(int(output[oc, y, x]) & 0xFF))
print(f"[L0] RTL-order output -> {rtl_hex_path}  ({OUT_CH*out_h*out_w} lines)")

print()
print("=" * 60)
print("  Layer 0 Golden Package Complete")
print("=" * 60)
print(f"  layer0_input.hex           {IN_CH}x{IMG_H}x{IMG_W}  = {IN_CH*IMG_H*IMG_W:>8,} bytes")
print(f"  layer0_weights_flat.hex    {OUT_CH}x{IN_CH}x{KH}x{KW}    = {OUT_CH*IN_CH*KH*KW:>8,} bytes  (pre-existing)")
print(f"  layer0_bias.hex            {OUT_CH} x INT32   = {OUT_CH*4:>8,} bytes  (pre-existing)")
print(f"  layer0_expected_output.hex {OUT_CH}x{out_h}x{out_w} = {OUT_CH*out_h*out_w:>8,} bytes  [oc][y][x]")
print(f"  layer0_expected_rtl_order  {out_h}x{out_w}x{OUT_CH} = {OUT_CH*out_h*out_w:>8,} bytes  [y][x][oc]")
print(f"  Requantizer: M={RQ_M}  shift={RQ_SH}  zp={RQ_ZP}")
print(f"  Activation:  LeakyReLU (slope 1/2^{LEAK_SHIFT} = {1/2**LEAK_SHIFT:.4f})")
