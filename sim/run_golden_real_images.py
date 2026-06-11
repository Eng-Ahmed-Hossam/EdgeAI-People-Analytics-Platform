#!/usr/bin/env python3
"""
run_golden_real_images.py

Run the complete Tiny-YOLOv3 golden model on real images (bus.jpg, zidane.jpg)
and produce:
  1. Detection tensors (L18 output, 255x4x4 INT8)
  2. Bounding boxes (raw, before NMS)
  3. NMS output
  4. Person count
  5. Annotated output images

All forward-pass arithmetic is bit-exact with the RTL:
  - INT8 quantized convolution
  - Bias INT32 accumulation
  - Requantizer (multiply-shift-clamp)
  - Leaky ReLU (>>3 for negatives)
  - MaxPool stride=2 and stride=1 darknet variant

Usage:
    python run_golden_real_images.py

Outputs written to D:/Graduation_Project/RTL/sim/golden_real_output/
"""

import os, sys, math
import numpy as np

# ── PIL/Pillow for image I/O ─────────────────────────────────────────────────
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    sys.exit("PIL not found — install with: pip install Pillow")

# ── Project paths ─────────────────────────────────────────────────────────────
SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
RTL_HEX     = os.path.join(SCRIPT_DIR, "rtl_hex")
OUT_DIR     = os.path.join(SCRIPT_DIR, "golden_real_output")
GOLDEN_DIR  = r"D:\Graduation_Project\Golden_Model"
IMG_DIR     = os.path.join(GOLDEN_DIR, "models", "yolov3", "data", "images")

IMAGES = {
    "bus":    os.path.join(IMG_DIR, "bus.jpg"),
    "zidane": os.path.join(IMG_DIR, "zidane.jpg"),
}
GROUND_TRUTH = {"bus": 4, "zidane": 2}  # known person counts

os.makedirs(OUT_DIR, exist_ok=True)

# ── Model constants ───────────────────────────────────────────────────────────
IMG_H, IMG_W = 128, 128
GRID_H, GRID_W = 4, 4
NUM_ANCHORS = 3
NUM_CLASSES = 80
ENTRY_SIZE  = 5 + NUM_CLASSES   # 85
TENSOR_SIZE = GRID_H * GRID_W * NUM_ANCHORS * ENTRY_SIZE  # 4080

CONF_THRESH = 0.25
NMS_THRESH  = 0.45
INT8_SCALE  = 1.0 / 16.0
LEAK_SHIFT  = 3
PE_ROWS     = 16

# Anchors: original 416px coords, scaled to 128px
ANCHORS = [
    (81 * 128/416,   82 * 128/416),
    (135 * 128/416, 169 * 128/416),
    (344 * 128/416, 319 * 128/416),
]

# COCO class names (first 10 for labels, rest abbreviated)
COCO_NAMES_FILE = os.path.join(GOLDEN_DIR, "models", "yolov3", "data", "coco.names")
try:
    with open(COCO_NAMES_FILE) as f:
        COCO_NAMES = [l.strip() for l in f if l.strip()]
except FileNotFoundError:
    COCO_NAMES = [f"cls{i}" for i in range(NUM_CLASSES)]
    COCO_NAMES[0] = "person"

# ── Requantizer params (from scheduler_rom_params.vh) ────────────────────────
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

LAYERS = [
    #  idx   type    in    out   k   pad   act      dim
    (   0, 'conv',   3,   16,  3,   1, 'leaky',  128),
    (   1, 'pool',  16,   16,  2,   0, 'none',   128),
    (   2, 'conv',  16,   32,  3,   1, 'leaky',   64),
    (   3, 'pool',  32,   32,  2,   0, 'none',    64),
    (   4, 'conv',  32,   64,  3,   1, 'leaky',   32),
    (   5, 'pool',  64,   64,  2,   0, 'none',    32),
    (   6, 'conv',  64,  128,  3,   1, 'leaky',   16),
    (   7, 'pool', 128,  128,  2,   0, 'none',    16),
    (   8, 'conv', 128,  256,  3,   1, 'leaky',    8),
    (   9, 'pool', 256,  256,  2,   0, 'none',     8),
    (  10, 'conv', 256,  256,  3,   1, 'leaky',    4),
    (  11, 'pool', 256,  256,  2,   1, 'none',     4),
    (  12, 'conv', 256,  256,  3,   1, 'leaky',    4),
    (  13, 'conv', 256,  256,  1,   0, 'leaky',    4),
    (  14, 'conv', 256,  256,  3,   1, 'leaky',    4),
    (  15, 'conv', 256,  255,  1,   0, 'none',     4),
    (  16, 'conv', 255,  128,  1,   0, 'leaky',    4),
    (  17, 'conv', 128,  256,  3,   1, 'leaky',    4),
    (  18, 'conv', 256,  255,  1,   0, 'none',     4),
]

# ── Weight / bias loading ─────────────────────────────────────────────────────

def load_flat_weights(lidx, out_ch, in_ch, kH, kW):
    path = os.path.join(RTL_HEX, f"layer{lidx}_weights_flat.hex")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing weight file: {path}")
    raw = np.array([int(l.strip(), 16) for l in open(path) if l.strip()], dtype=np.uint8)
    return raw.view(np.int8).reshape(out_ch, in_ch, kH, kW)


def load_biases(lidx, out_ch):
    path = os.path.join(RTL_HEX, f"layer{lidx}_bias.hex")
    if not os.path.exists(path):
        raise FileNotFoundError(f"Missing bias file: {path}")
    raw = np.array([int(l.strip(), 16) for l in open(path) if l.strip()], dtype=np.uint32)
    return raw[:out_ch].view(np.int32)

# ── Forward-pass operators ────────────────────────────────────────────────────

def quantized_conv(inp, weights, biases, pad, M, shift, zp, act):
    out_ch, in_ch, kH, kW = weights.shape
    C, H, W = inp.shape
    padded = np.pad(inp, ((0,0),(pad,pad),(pad,pad))).astype(np.int64)
    acc = np.zeros((out_ch, H, W), dtype=np.int64)
    for ic in range(in_ch):
        for ky in range(kH):
            for kx in range(kW):
                patch = padded[ic, ky:ky+H, kx:kx+W]
                for oc in range(out_ch):
                    acc[oc] += patch * int(weights[oc, ic, ky, kx])
    acc += biases.astype(np.int64).reshape(out_ch, 1, 1)
    shifted = (acc * np.int64(M)) >> np.int64(shift)
    clamped = np.clip(shifted + np.int64(zp), -128, 127).astype(np.int8)
    if act == 'leaky':
        c16 = clamped.astype(np.int16)
        return np.where(c16 >= 0, c16, c16 >> LEAK_SHIFT).astype(np.int8)
    return clamped


def maxpool_stride2(feat):
    C, H, W = feat.shape
    out_h = H // 2
    out_w = W // 2
    out = np.full((C, out_h, out_w), np.iinfo(np.int8).min, dtype=np.int8)
    for oy in range(out_h):
        for ox in range(out_w):
            out[:, oy, ox] = feat[:, oy*2:oy*2+2, ox*2:ox*2+2].reshape(C, -1).max(axis=1)
    return out


def maxpool_stride1_darknet(feat):
    C, H, W = feat.shape
    padded = np.full((C, H+1, W+1), np.iinfo(np.int8).min, dtype=np.int8)
    padded[:, :H, :W] = feat
    return np.maximum(
        np.maximum(padded[:, :-1, :-1], padded[:, :-1, 1:]),
        np.maximum(padded[:, 1:,  :-1], padded[:, 1:,  1:])
    ).astype(np.int8)

# ── YOLO post-processing ──────────────────────────────────────────────────────

def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x.astype(np.float32)))


def yolo_decode(tensor_chyx):
    """tensor_chyx: [255, 4, 4] INT8.  Returns list of (x1,y1,x2,y2,conf,cls)."""
    t = tensor_chyx.astype(np.float32) * INT8_SCALE
    cell_w = IMG_W / GRID_W
    cell_h = IMG_H / GRID_H
    dets = []
    for a, (aw, ah) in enumerate(ANCHORS):
        base = a * ENTRY_SIZE
        obj_conf = sigmoid(t[base+4])                      # [4,4]
        for gy in range(GRID_H):
            for gx in range(GRID_W):
                obj = obj_conf[gy, gx]
                if obj < CONF_THRESH:
                    continue
                cls_logits = t[base+5 : base+5+NUM_CLASSES, gy, gx]
                cls_probs  = sigmoid(cls_logits)
                best_cls   = int(np.argmax(cls_probs))
                score      = obj * cls_probs[best_cls]
                if score < CONF_THRESH:
                    continue
                tx = t[base+0, gy, gx]
                ty = t[base+1, gy, gx]
                tw = t[base+2, gy, gx]
                th = t[base+3, gy, gx]
                bx = (sigmoid(tx) + gx) * cell_w
                by = (sigmoid(ty) + gy) * cell_h
                bw = aw * math.exp(float(tw))
                bh = ah * math.exp(float(th))
                dets.append((bx - bw/2, by - bh/2, bx + bw/2, by + bh/2,
                              float(score), best_cls))
    return dets


def iou(a, b):
    ix1 = max(a[0], b[0]); iy1 = max(a[1], b[1])
    ix2 = min(a[2], b[2]); iy2 = min(a[3], b[3])
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0.0


def nms(dets, iou_thresh):
    by_cls = {}
    for d in dets:
        c = d[5]
        by_cls.setdefault(c, []).append(d)
    out = []
    for c, ds in by_cls.items():
        ds.sort(key=lambda x: -x[4])
        keep = []
        while ds:
            best = ds.pop(0)
            keep.append(best)
            ds = [d for d in ds if iou(best, d) <= iou_thresh]
        out.extend(keep)
    return out

# ── Image preprocessing ───────────────────────────────────────────────────────

def preprocess(img_path):
    """Load image, resize to 128×128, convert to quantized INT8 [C, H, W]."""
    img = Image.open(img_path).convert("RGB").resize((IMG_W, IMG_H), Image.BILINEAR)
    arr = np.array(img, dtype=np.int16)   # [H, W, 3], uint8
    quant = (arr - 128).astype(np.int8)   # pixel − 128 → INT8
    return quant.transpose(2, 0, 1)       # [3, 128, 128]

# ── Full forward pass ─────────────────────────────────────────────────────────

def forward(inp):
    current = inp
    for (lidx, ltype, in_ch, out_ch, k, pad, act, dim) in LAYERS:
        M, shift, zp = RQ[lidx]
        if ltype == 'conv':
            W_layer = load_flat_weights(lidx, out_ch, in_ch, k, k)
            B_layer = load_biases(lidx, out_ch)
            current = quantized_conv(current, W_layer, B_layer, pad, M, shift, zp, act)
        else:  # pool
            if lidx == 11:
                current = maxpool_stride1_darknet(current)
            else:
                current = maxpool_stride2(current)
        print(f"  L{lidx:02d} {ltype:4s} → shape {current.shape}  "
              f"range [{current.min():4d}, {current.max():4d}]")
    return current   # [255, 4, 4] INT8

# ── Draw annotations ─────────────────────────────────────────────────────────

def draw_boxes(img_path, dets, scale_w, scale_h, tag):
    img = Image.open(img_path).convert("RGB")
    draw = ImageDraw.Draw(img)
    for (x1, y1, x2, y2, conf, cls) in dets:
        lbl = COCO_NAMES[cls] if cls < len(COCO_NAMES) else f"cls{cls}"
        color = (255, 0, 0) if cls == 0 else (0, 200, 0)
        draw.rectangle([x1*scale_w, y1*scale_h, x2*scale_w, y2*scale_h],
                        outline=color, width=2)
        draw.text((x1*scale_w + 2, y1*scale_h + 2),
                   f"{lbl} {conf:.2f}", fill=color)
    out_path = os.path.join(OUT_DIR, f"{tag}_annotated.jpg")
    img.save(out_path)
    print(f"  Annotated: {out_path}")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 62)
    print("  Golden Model — Real Image Inference")
    print("=" * 62)

    results = {}

    for name, img_path in IMAGES.items():
        print(f"\n{'─'*60}")
        print(f"  Image: {name}  ({img_path})")
        print(f"{'─'*60}")

        if not os.path.exists(img_path):
            print(f"  [SKIP] File not found: {img_path}")
            continue

        # Load original size for annotation scaling
        orig = Image.open(img_path)
        orig_w, orig_h = orig.size
        scale_w = orig_w / IMG_W
        scale_h = orig_h / IMG_H

        # Preprocess
        inp = preprocess(img_path)
        print(f"  Input quantized: shape={inp.shape}  "
              f"range=[{inp.min()}, {inp.max()}]")

        # Forward pass
        tensor_chyx = forward(inp)    # [255, 4, 4]

        # Save tensor as hex (channel-major, same layout as RTL scatter output)
        tensor_path = os.path.join(OUT_DIR, f"{name}_tensor_l18.hex")
        with open(tensor_path, 'w') as f:
            for c in range(255):
                for y in range(4):
                    for x in range(4):
                        f.write(f"{int(tensor_chyx[c, y, x]) & 0xFF:02x}\n")
        print(f"  Tensor saved: {tensor_path}  (4080 bytes)")

        # Decode
        raw_dets = yolo_decode(tensor_chyx)
        print(f"  Raw detections (pre-NMS): {len(raw_dets)}")
        for d in raw_dets[:5]:
            lbl = COCO_NAMES[d[5]] if d[5] < len(COCO_NAMES) else f"cls{d[5]}"
            print(f"    [{lbl}] conf={d[4]:.3f}  box=({d[0]:.1f},{d[1]:.1f},"
                  f"{d[2]:.1f},{d[3]:.1f})")

        # NMS
        nms_dets = nms(raw_dets, NMS_THRESH)
        print(f"  Post-NMS detections: {len(nms_dets)}")

        # Person count
        persons = sum(1 for d in nms_dets if d[5] == 0)
        gt = GROUND_TRUTH.get(name, "?")
        match = "✓" if persons == gt else "✗"
        print(f"  Person count: {persons}  (ground truth: {gt})  {match}")

        # Annotated image
        draw_boxes(img_path, nms_dets, scale_w, scale_h, name)

        # Save detection report
        rpt_path = os.path.join(OUT_DIR, f"{name}_detections.txt")
        with open(rpt_path, 'w') as f:
            f.write(f"Image: {img_path}\n")
            f.write(f"Post-NMS detections: {len(nms_dets)}\n")
            f.write(f"Person count: {persons}  (GT={gt})\n\n")
            for d in nms_dets:
                lbl = COCO_NAMES[d[5]] if d[5] < len(COCO_NAMES) else f"cls{d[5]}"
                f.write(f"{lbl:20s}  conf={d[4]:.4f}  "
                        f"x1={d[0]:6.1f} y1={d[1]:6.1f} "
                        f"x2={d[2]:6.1f} y2={d[3]:6.1f}\n")

        results[name] = {
            "tensor_path":  tensor_path,
            "raw_dets":     len(raw_dets),
            "nms_dets":     len(nms_dets),
            "person_count": persons,
            "ground_truth": gt,
            "correct":      (persons == gt),
        }

    # Summary
    print(f"\n{'='*62}")
    print("  SUMMARY")
    print(f"{'='*62}")
    for name, r in results.items():
        status = "PASS" if r["correct"] else "FAIL"
        print(f"  [{status}] {name:10s}  "
              f"persons={r['person_count']} (GT={r['ground_truth']})  "
              f"raw={r['raw_dets']}  nms={r['nms_dets']}")
    print(f"{'='*62}")
    print(f"  Output directory: {OUT_DIR}")


if __name__ == "__main__":
    main()
