# post/nms.py
from typing import List, Dict
import numpy as np

def iou(box_a, box_b):
    """
    IoU for boxes in format x,y,w,h (top-left)
    """
    ax1, ay1, aw, ah = box_a["x"], box_a["y"], box_a["w"], box_a["h"]
    bx1, by1, bw, bh = box_b["x"], box_b["y"], box_b["w"], box_b["h"]
    ax2, ay2 = ax1 + aw, ay1 + ah
    bx2, by2 = bx1 + bw, by1 + bh

    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)

    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    area_a = aw * ah
    area_b = bw * bh
    union = area_a + area_b - inter_area
    return inter_area / union if union > 0 else 0.0


def non_max_suppression(detections: List[Dict], iou_threshold: float = 0.45):
    """
    Standard NMS.
    detections: list of dicts {x,y,w,h,confidence}
    Returns filtered list.
    """
    if not detections:
        return []

    # Sort by confidence descending
    dets = sorted(detections, key=lambda d: d["confidence"], reverse=True)
    keep = []
    while dets:
        best = dets.pop(0)
        keep.append(best)
        remaining = []
        for d in dets:
            if iou(best, d) < iou_threshold:
                remaining.append(d)
        dets = remaining
    return keep
