#!/usr/bin/env python3
"""
golden_network.py

Full 19-layer Tiny-YOLO bit-exact forward pass.

Generates for every CONV layer N:
  layerN_input.hex            [ch][y][x] layout, 1 byte/line
  layerN_expected_rtl_order.hex  [oy][ox][oc] layout, 1 byte/line

Uses scheduler ROM parameters (not meta.json) as authoritative source:
  - 1x1 conv layers (L13, L15, L16, L18): pad=0
  - 3x3 conv layers: pad=1
  - L15, L18: ACT_NONE (linear, no activation)
  - All others with activation: leaky (>> 3 for negatives)
"""

import os
import json
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_HEX    = os.path.join(SCRIPT_DIR, "rtl_hex")
LEAK_SHIFT = 3

# ---------------------------------------------------------------------------
# Requantizer params from scheduler_rom_params.vh / export_summary.json
# ---------------------------------------------------------------------------
RQ = [
    (40370, 23, 0),   # L0
    (    0,  0, 0),   # L1  pool
    (50412, 24, 0),   # L2
    (    0,  0, 0),   # L3  pool
    (36616, 24, 0),   # L4
    (    0,  0, 0),   # L5  pool
    (51468, 25, 0),   # L6
    (    0,  0, 0),   # L7  pool
    (39603, 25, 0),   # L8
    (    0,  0, 0),   # L9  pool
    (54718, 26, 0),   # L10
    (    0,  0, 0),   # L11 pool
    (56779, 26, 0),   # L12
    (51601, 26, 0),   # L13
    (62723, 26, 0),   # L14
    (41974, 25, 0),   # L15
    (41738, 25, 0),   # L16
    (22027, 31, 0),   # L17
    (62657, 25, 0),   # L18
]

# Layer configuration (index, type, in_ch, out_ch, kernel, pad, act, img_dim)
# type: 'conv' or 'pool'
# pad: effective pad used in RTL (matches layer_scheduler.v rom_pad)
# act: 'leaky', 'none'
LAYERS = [
    # idx  type    in   out  k  pad  act      dim
    (  0, 'conv',   3,  16, 3,  1, 'leaky',  128),
    (  1, 'pool',  16,  16, 2,  0, 'none',   128),  # stride=2 → 64
    (  2, 'conv',  16,  32, 3,  1, 'leaky',   64),
    (  3, 'pool',  32,  32, 2,  0, 'none',    64),  # stride=2 → 32
    (  4, 'conv',  32,  64, 3,  1, 'leaky',   32),
    (  5, 'pool',  64,  64, 2,  0, 'none',    32),  # stride=2 → 16
    (  6, 'conv',  64, 128, 3,  1, 'leaky',   16),
    (  7, 'pool', 128, 128, 2,  0, 'none',    16),  # stride=2 → 8
    (  8, 'conv', 128, 256, 3,  1, 'leaky',    8),
    (  9, 'pool', 256, 256, 2,  0, 'none',     8),  # stride=2 → 4
    ( 10, 'conv', 256, 256, 3,  1, 'leaky',    4),
    ( 11, 'pool', 256, 256, 2,  1, 'none',     4),  # stride=1, darknet-pad → 4
    ( 12, 'conv', 256, 256, 3,  1, 'leaky',    4),
    ( 13, 'conv', 256, 256, 1,  0, 'leaky',    4),
    ( 14, 'conv', 256, 256, 3,  1, 'leaky',    4),
    ( 15, 'conv', 256, 255, 1,  0, 'none',     4),  # linear, no activation
    ( 16, 'conv', 255, 128, 1,  0, 'leaky',    4),
    ( 17, 'conv', 128, 256, 3,  1, 'leaky',    4),
    ( 18, 'conv', 256, 255, 1,  0, 'none',     4),  # linear, no activation
]


def load_flat_weights(layer_idx, out_ch, in_ch, kH, kW):
    """Load flat weights [out_ch, in_ch, kH, kW] from layer{N}_weights_flat.hex"""
    path = os.path.join(RTL_HEX, f"layer{layer_idx}_weights_flat.hex")
    with open(path) as f:
        raw = np.array([int(ln.strip(), 16) for ln in f if ln.strip()], dtype=np.uint8)
    w = raw.view(np.int8).reshape(out_ch, in_ch, kH, kW)
    assert w.shape == (out_ch, in_ch, kH, kW), f"L{layer_idx} weight shape mismatch"
    return w


def load_biases(layer_idx, out_ch):
    """Load biases [out_ch] INT32 from layer{N}_bias.hex (8 hex chars per line)"""
    path = os.path.join(RTL_HEX, f"layer{layer_idx}_bias.hex")
    with open(path) as f:
        raw = np.array([int(ln.strip(), 16) for ln in f if ln.strip()], dtype=np.uint32)
    b = raw[:out_ch].view(np.int32)
    assert len(b) == out_ch, f"L{layer_idx} bias count mismatch"
    return b


def quantized_conv(inp, weights, biases, pad, M, shift, zp, act):
    """
    Bit-exact INT8 quantized convolution matching RTL pipeline:
      acc(INT64) = conv(inp, weights) + bias
      rq = clamp((acc * M) >> shift + zp, -128, 127)
      act: leaky → rq if rq>=0 else rq>>3
           none  → rq as-is
    """
    out_ch, in_ch, kH, kW = weights.shape
    C, H, W = inp.shape
    assert C == in_ch, f"Channel mismatch: inp={C}, weights in_ch={in_ch}"

    out_h = H   # same-padding: H + 2*pad - kH + 1 = H for 3x3,pad=1
    out_w = W   # and 1x1,pad=0

    padded = np.pad(inp, ((0, 0), (pad, pad), (pad, pad))).astype(np.int64)
    acc = np.zeros((out_ch, out_h, out_w), dtype=np.int64)

    for ic in range(in_ch):
        for ky in range(kH):
            for kx in range(kW):
                patch = padded[ic, ky:ky+out_h, kx:kx+out_w]
                for oc in range(out_ch):
                    acc[oc] += patch * int(weights[oc, ic, ky, kx])

    acc += biases.astype(np.int64).reshape(out_ch, 1, 1)

    product = acc * np.int64(M)
    shifted = product >> np.int64(shift)
    with_zp  = shifted + np.int64(zp)
    clamped  = np.clip(with_zp, -128, 127).astype(np.int8)

    if act == 'leaky':
        c16 = clamped.astype(np.int16)
        output = np.where(c16 >= 0, c16, c16 >> LEAK_SHIFT).astype(np.int8)
    else:  # 'none' / linear
        output = clamped

    return output


def maxpool_stride2(feat, kernel=2):
    """2x2 max pool stride=2, no padding. Output: [C, H/2, W/2]"""
    C, H, W = feat.shape
    out_h = (H - kernel) // 2 + 1
    out_w = (W - kernel) // 2 + 1
    out = np.full((C, out_h, out_w), np.iinfo(np.int8).min, dtype=np.int8)
    for oy in range(out_h):
        for ox in range(out_w):
            window = feat[:, oy*2:oy*2+kernel, ox*2:ox*2+kernel]
            out[:, oy, ox] = window.reshape(C, -1).max(axis=1)
    return out


def maxpool_stride1_darknet(feat):
    """
    2x2 max pool stride=1 with darknet asymmetric padding (right+bottom by 1).
    Preserves spatial dimension: [C, H, W] -> [C, H, W].
    """
    C, H, W = feat.shape
    # Pad right and bottom with INT8_MIN
    padded = np.full((C, H+1, W+1), np.iinfo(np.int8).min, dtype=np.int8)
    padded[:, :H, :W] = feat
    out = np.maximum(
        np.maximum(padded[:, :-1, :-1], padded[:, :-1, 1:]),
        np.maximum(padded[:, 1:,  :-1], padded[:, 1:,  1:])
    )
    return out.astype(np.int8)


def write_input_hex(feat, path):
    """Write [C, H, W] feature map in [c][y][x] order, 2-hex chars per line."""
    C, H, W = feat.shape
    with open(path, 'w') as f:
        for c in range(C):
            for y in range(H):
                for x in range(W):
                    f.write("{:02x}\n".format(int(feat[c, y, x]) & 0xFF))


PE_ROWS_PY = 16  # Must match PE_ROWS in edgeai_defs.vh

def write_rtl_order_hex(feat, path):
    """
    Write [C, H, W] feature map in RTL stream order.

    The cnn_core FSM outputs in [oc_tile][oy][ox][row] order:
      for each oc_tile (tile-0 first, then tile-1, ...):
        for each spatial position (oy row-major):
          emit PE_ROWS consecutive output channels for that tile.

    This matches the SINGLE-COLUMN PE array design where all active rows of
    one tile are emitted before moving to the next tile.  For single-tile
    layers (C <= PE_ROWS) this reduces to the familiar [y][x][c] order.
    """
    C, H, W = feat.shape
    num_tiles = (C + PE_ROWS_PY - 1) // PE_ROWS_PY
    with open(path, 'w') as f:
        for tile in range(num_tiles):
            oc_base    = tile * PE_ROWS_PY
            active_rows = min(PE_ROWS_PY, C - oc_base)
            for y in range(H):
                for x in range(W):
                    for row in range(active_rows):
                        f.write("{:02x}\n".format(int(feat[oc_base + row, y, x]) & 0xFF))


# ---------------------------------------------------------------------------
# Generate test input (same pattern as golden_layer0.py)
# ---------------------------------------------------------------------------
IMG_H, IMG_W = 128, 128
IN_CH = 3
ys = np.arange(IMG_H, dtype=np.int32)
xs = np.arange(IMG_W, dtype=np.int32)
YY, XX = np.meshgrid(ys, xs, indexing='ij')

inp = np.zeros((IN_CH, IMG_H, IMG_W), dtype=np.int8)
for c in range(IN_CH):
    raw_c = (int(c) * 43 + YY * 7 + XX * 3) % 256
    inp[c] = (raw_c - 128).astype(np.int8)

print(f"Test input: shape={inp.shape}  range=[{inp.min()}, {inp.max()}]")

# ---------------------------------------------------------------------------
# Forward pass through all 19 layers
# ---------------------------------------------------------------------------
current = inp
stats = []

for (idx, ltype, in_ch, out_ch, k, pad, act, dim) in LAYERS:
    M, shift, zp = RQ[idx]

    if ltype == 'conv':
        print(f"\n[L{idx}] Conv  {in_ch}->{out_ch}  {k}x{k}  pad={pad}  {act}  dim={dim}  rq=({M},{shift},{zp})")

        # Save input hex for this conv layer
        inp_path = os.path.join(RTL_HEX, f"layer{idx}_input.hex")
        write_input_hex(current, inp_path)
        print(f"  Input  -> {inp_path}  ({in_ch}×{dim}×{dim} = {in_ch*dim*dim} bytes)")

        # Load weights and biases
        weights = load_flat_weights(idx, out_ch, in_ch, k, k)
        biases  = load_biases(idx, out_ch)

        # Compute bit-exact output
        output = quantized_conv(current, weights, biases, pad, M, shift, zp, act)
        print(f"  Output range: [{output.min()}, {output.max()}]  nonzero: {np.count_nonzero(output)}/{output.size}")

        # Save RTL-order golden (stream order [oy][ox][oc] for conv testbench)
        rtl_path = os.path.join(RTL_HEX, f"layer{idx}_expected_rtl_order.hex")
        write_rtl_order_hex(output, rtl_path)
        print(f"  Golden -> {rtl_path}  ({out_ch}x{dim}x{dim} = {out_ch*dim*dim} bytes)")

        # Save channel-major output (pool input hex for pool testbench)
        chyx_path = os.path.join(RTL_HEX, f"layer{idx}_output_chyx.hex")
        write_input_hex(output, chyx_path)
        print(f"  ChYX   -> {chyx_path}  (pool input format)")

        stats.append((idx, out_ch * dim * dim))
        current = output

    else:  # pool
        stride = 2 if pad == 0 else 1
        print(f"[L{idx}] Pool  {k}×{k}  stride={stride}  {'darknet-pad' if stride==1 else 'no-pad'}")
        if stride == 2:
            current = maxpool_stride2(current, kernel=k)
        else:
            current = maxpool_stride1_darknet(current)
        print(f"  Output shape: {current.shape}")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
print("\n" + "="*60)
print("  Golden Network Pass Complete")
print("="*60)
print(f"  Layer  |  Outputs  | File")
print(f"  -------|-----------|------")
for (idx, n) in stats:
    print(f"  L{idx:2d}    | {n:8d}  | layer{idx}_input.hex + layer{idx}_expected_rtl_order.hex")

print(f"\n  Final output (L18): shape={current.shape}  range=[{current.min()}, {current.max()}]")
print("="*60)
