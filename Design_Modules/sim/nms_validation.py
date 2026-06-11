#!/usr/bin/env python3
"""
nms_validation.py  — Phase 13: Post-processing & NMS Validation

Verifies that the RTL detection tensor (captured from simulation) matches
the Python golden model's detection tensor, and that NMS produces the
same person count from both.

Architecture: Tiny-YOLOv3, 128x128 input, single 4x4 output head.
  - 6 anchors total; large head uses mask [3,4,5]
  - 80 COCO classes; person = class 0
  - L15 and L18 each produce [255, 4, 4] output (3 anchors × 85 values)

Running modes:
  python nms_validation.py          -- compare golden vs golden (sanity check)
  python nms_validation.py rtl      -- compare golden vs RTL hex capture

The RTL capture file 'rtl_hex/layer18_rtl_capture.hex' must be present
when running in 'rtl' mode (captured from tb_conv_layers RTL simulation).
"""

import os
import sys
import math
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
RTL_HEX    = os.path.join(SCRIPT_DIR, "rtl_hex")

# ── Model constants ──────────────────────────────────────────────────────────
IMG_W,   IMG_H   = 128, 128
GRID_W,  GRID_H  = 4, 4
NUM_CLASSES      = 80
PERSON_CLS       = 0

# YOLOv3-tiny original anchors (from yolov3-tiny.cfg, trained at 416x416):
#   all: [(10,14), (23,27), (37,58), (81,82), (135,169), (344,319)]
# Large output head uses mask 3,4,5 → anchors [3][4][5]:
#   (81,82), (135,169), (344,319)
# Scale to our 128x128 input:
SCALE = IMG_W / 416.0
ANCHORS_LARGE_416 = [(81, 82), (135, 169), (344, 319)]
ANCHORS = [(w * SCALE, h * SCALE) for (w, h) in ANCHORS_LARGE_416]
NUM_ANCHORS = len(ANCHORS)   # 3

# Detection hyperparameters
CONF_THRESH = 0.25
NMS_THRESH  = 0.45

# ── INT8 output scale ────────────────────────────────────────────────────────
# L18 requantizer: (M=62657, shift=25, zp=0)
# Effective multiplier = M / 2^shift = 62657 / 33554432 ≈ 1.867e-3
# This is the per-value scale relative to the layer's input range.
# For YOLO decode we need the float activation before quantization.
# We use the int8 output directly with a learned/calibrated scale factor.
# Since we're comparing RTL vs Python (not float vs int8), the exact scale
# cancels out as long as both use the same value.  We treat int8 as proxy.
# A scale of 1/16 puts the typical int8 range [-16..127] into [-1..8] which
# is reasonable for raw YOLO logits before sigmoid.
INT8_SCALE = 1.0 / 16.0


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def load_rtl_order_hex(path, out_ch, h, w):
    """
    Load a tile-major RTL stream hex file into [out_ch, h, w] numpy array.
    Stream order: [oc_tile][y][x][row] (same as cnn_core output stream).
    """
    pe_rows = 16
    with open(path) as f:
        flat = np.array([int(ln.strip(), 16) for ln in f if ln.strip()],
                        dtype=np.uint8).view(np.int8)

    result = np.zeros((out_ch, h, w), dtype=np.int8)
    num_tiles = (out_ch + pe_rows - 1) // pe_rows
    idx = 0
    for tile in range(num_tiles):
        oc_base = tile * pe_rows
        active  = min(pe_rows, out_ch - oc_base)
        for y in range(h):
            for x in range(w):
                for row in range(active):
                    result[oc_base + row, y, x] = flat[idx]
                    idx += 1
    return result


def decode_yolo_head(tensor_int8, anchors, grid_h, grid_w, img_h, img_w):
    """
    Decode a YOLOv3 detection head tensor.

    tensor_int8: [num_anchors*(5+num_classes), grid_h, grid_w] INT8 numpy array
    Returns list of dicts: {box:[x1,y1,x2,y2], conf, cls_id, cls_conf}
    Coordinates in [0,1] relative to image.
    """
    C = tensor_int8.shape[0]
    assert C == NUM_ANCHORS * (5 + NUM_CLASSES), f"Expected {NUM_ANCHORS*(5+NUM_CLASSES)} channels, got {C}"

    # Scale int8 to approximate float logits
    tensor = tensor_int8.astype(np.float32) * INT8_SCALE
    # Reshape: [num_anchors, 5+num_classes, grid_h, grid_w]
    tensor = tensor.reshape(NUM_ANCHORS, 5 + NUM_CLASSES, grid_h, grid_w)

    detections = []
    for a, (aw, ah) in enumerate(anchors):
        tx = tensor[a, 0]   # [grid_h, grid_w]
        ty = tensor[a, 1]
        tw = tensor[a, 2]
        th = tensor[a, 3]
        obj = sigmoid(tensor[a, 4])
        cls_raw = tensor[a, 5:]  # [num_classes, grid_h, grid_w]
        cls_probs = sigmoid(cls_raw)

        for gy in range(grid_h):
            for gx in range(grid_w):
                conf = float(obj[gy, gx])
                if conf < CONF_THRESH:
                    continue

                # Box center in [0,1] relative to image
                cx = (sigmoid(tx[gy, gx]) + gx) / grid_w
                cy = (sigmoid(ty[gy, gx]) + gy) / grid_h
                bw = (aw * math.exp(float(tw[gy, gx]))) / img_w
                bh = (ah * math.exp(float(th[gy, gx]))) / img_h

                # Clamp to [0,1]
                x1 = max(0.0, cx - bw / 2)
                y1 = max(0.0, cy - bh / 2)
                x2 = min(1.0, cx + bw / 2)
                y2 = min(1.0, cy + bh / 2)

                cls_scores = cls_probs[:, gy, gx]
                cls_id = int(np.argmax(cls_scores))
                cls_conf = float(cls_scores[cls_id])
                score = conf * cls_conf

                if score >= CONF_THRESH:
                    detections.append({
                        "box":      [x1, y1, x2, y2],
                        "conf":     conf,
                        "cls_id":   cls_id,
                        "score":    score,
                        "anchor_a": a,
                        "grid_yx":  (gy, gx)
                    })
    return detections


def iou(box_a, box_b):
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    return inter / (area_a + area_b - inter + 1e-9)


def nms(detections, iou_thresh):
    """Per-class non-maximum suppression."""
    if not detections:
        return []
    # Separate by class
    classes = set(d["cls_id"] for d in detections)
    kept = []
    for c in classes:
        cls_dets = [d for d in detections if d["cls_id"] == c]
        cls_dets.sort(key=lambda d: -d["score"])
        while cls_dets:
            best = cls_dets.pop(0)
            kept.append(best)
            cls_dets = [d for d in cls_dets
                        if iou(best["box"], d["box"]) < iou_thresh]
    return kept


def count_persons(detections):
    return sum(1 for d in detections if d["cls_id"] == PERSON_CLS)


def main():
    use_rtl = len(sys.argv) > 1 and sys.argv[1] == "rtl"

    print("=" * 60)
    print("  NMS Validation — Phase 13")
    print("  Tiny-YOLOv3 128x128, 4x4 output head (L18)")
    print(f"  Anchors (scaled to 128px): {ANCHORS}")
    print(f"  Conf threshold: {CONF_THRESH}  NMS IoU: {NMS_THRESH}")
    print("=" * 60)

    # ── Load Python golden tensor ─────────────────────────────────────────
    golden_path = os.path.join(RTL_HEX, "layer18_expected_rtl_order.hex")
    if not os.path.exists(golden_path):
        print(f"ERROR: Golden file not found: {golden_path}")
        print("Run golden_network.py first to generate it.")
        sys.exit(1)

    golden_tensor = load_rtl_order_hex(golden_path, 255, GRID_H, GRID_W)
    print(f"\n[Python Golden] L18 tensor shape: {golden_tensor.shape}")
    print(f"  range=[{golden_tensor.min()}, {golden_tensor.max()}]")
    print(f"  nonzero: {np.count_nonzero(golden_tensor)} / {golden_tensor.size}")

    golden_dets = decode_yolo_head(golden_tensor, ANCHORS, GRID_H, GRID_W, IMG_H, IMG_W)
    golden_after_nms = nms(golden_dets, NMS_THRESH)
    golden_persons = count_persons(golden_after_nms)
    print(f"\n  Raw detections (pre-NMS): {len(golden_dets)}")
    print(f"  After NMS:                {len(golden_after_nms)}")
    print(f"  Person detections:        {golden_persons}")

    # ── Load RTL tensor (if requested) ────────────────────────────────────
    if use_rtl:
        rtl_path = os.path.join(RTL_HEX, "layer18_rtl_capture.hex")
        if not os.path.exists(rtl_path):
            print(f"\nERROR: RTL capture not found: {rtl_path}")
            print("Run tb_conv_layers simulation and capture L18 output first.")
            sys.exit(1)

        rtl_tensor = load_rtl_order_hex(rtl_path, 255, GRID_H, GRID_W)
        print(f"\n[RTL Capture] L18 tensor shape: {rtl_tensor.shape}")
        print(f"  range=[{rtl_tensor.min()}, {rtl_tensor.max()}]")

        # Byte-exact comparison
        mismatches = int(np.sum(golden_tensor != rtl_tensor))
        print(f"\n  Tensor mismatches: {mismatches} / {golden_tensor.size}")

        rtl_dets = decode_yolo_head(rtl_tensor, ANCHORS, GRID_H, GRID_W, IMG_H, IMG_W)
        rtl_after_nms = nms(rtl_dets, NMS_THRESH)
        rtl_persons = count_persons(rtl_after_nms)
        print(f"  RTL person detections: {rtl_persons}")

        print("\n" + "=" * 60)
        if mismatches == 0 and golden_persons == rtl_persons:
            print("  *** PASS: RTL detection tensor matches Python golden ***")
            print(f"  Person count: {rtl_persons} (Python={golden_persons}, RTL={rtl_persons})")
        else:
            print("  *** FAIL ***")
            if mismatches != 0:
                print(f"  Tensor mismatch: {mismatches} bytes differ")
            if golden_persons != rtl_persons:
                print(f"  Person count mismatch: Python={golden_persons}, RTL={rtl_persons}")
        print("=" * 60)
    else:
        print("\n[Golden-only mode]  Pass 'rtl' argument to compare against RTL capture.")
        print("\n" + "=" * 60)
        if golden_persons == 0:
            print("  No persons detected in synthetic test pattern (expected for noise input).")
        else:
            print(f"  Python golden detects {golden_persons} person(s).")
        print("=" * 60)


if __name__ == "__main__":
    main()
