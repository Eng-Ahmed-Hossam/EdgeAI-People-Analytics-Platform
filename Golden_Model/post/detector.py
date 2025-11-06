import numpy as np
from typing import List, Dict, Tuple

CLASSES = ["person", "car", "motorcycle"]  # extendable list of classes

def find_local_peaks(feature_map: np.ndarray, threshold: float = 0.2, neighborhood: int = 1) -> List[Tuple[int,int,float]]:
    H, W = feature_map.shape
    peaks = []
    for y in range(H):
        for x in range(W):
            val = float(feature_map[y, x])
            if val < threshold:
                continue
            y0 = max(0, y - neighborhood)
            y1 = min(H, y + neighborhood + 1)
            x0 = max(0, x - neighborhood)
            x1 = min(W, x + neighborhood + 1)
            window = feature_map[y0:y1, x0:x1]
            if val >= float(window.max()):
                peaks.append((y, x, val))
    return peaks

def pooled_to_image_coords(peak_y: int, peak_x: int, pooled_shape: Tuple[int,int],
                           input_image_shape: Tuple[int,int], pool_stride: int = 2) -> Tuple[int,int]:
    pooled_h, pooled_w = pooled_shape
    img_h, img_w = input_image_shape
    scale_y = img_h / pooled_h
    scale_x = img_w / pooled_w
    center_y = int((peak_y + 0.5) * scale_y)
    center_x = int((peak_x + 0.5) * scale_x)
    return center_y, center_x

def cluster_peaks(peaks, cluster_radius=1):
    """Greedy clustering of peaks (y,x,val). Returns clustered centers (y,x,val)."""
    if not peaks:
        return []
    peaks_sorted = sorted(peaks, key=lambda p: p[2], reverse=True)
    used = [False] * len(peaks_sorted)
    clusters = []
    for i, (y, x, v) in enumerate(peaks_sorted):
        if used[i]:
            continue
        members = [(y, x, v)]
        used[i] = True
        for j, (y2, x2, v2) in enumerate(peaks_sorted[i+1:], start=i+1):
            if used[j]:
                continue
            if abs(y2 - y) <= cluster_radius and abs(x2 - x) <= cluster_radius:
                members.append((y2, x2, v2))
                used[j] = True
        # weighted average center
        ws = [m[2] for m in members]
        ys = [m[0] * m[2] for m in members]
        xs = [m[1] * m[2] for m in members]
        y_avg = int(sum(ys) / sum(ws))
        x_avg = int(sum(xs) / sum(ws))
        v_avg = float(sum(ws) / len(ws))
        clusters.append((y_avg, x_avg, v_avg))
    return clusters

def detect_from_pooled_improved(feature_map: np.ndarray,
                                image_shape: Tuple[int,int],
                                min_conf: float = 0.25,
                                neighborhood: int = 1,
                                fixed_box_size: Tuple[int,int] = None,
                                border_ignore_frac: float = 0.05,
                                min_box_area: int = 400,
                                top_k: int = 20,
                                cluster_radius: int = 1) -> List[Dict]:
    """
    Improved detector that:
    - normalizes pooled map
    - uses adaptive threshold (relative to max)
    - clusters nearby peaks
    - ignores border peaks
    - filters tiny boxes
    - returns top_k boxes
    - assigns a class label
    """
    pooled_h, pooled_w = feature_map.shape
    pmin, pmax = float(feature_map.min()), float(feature_map.max())
    if pmax - pmin > 0:
        fmap = (feature_map - pmin) / (pmax - pmin)
    else:
        fmap = feature_map * 0.0

    adaptive_thr = max(min_conf, 0.2 * float(fmap.max()))
    peaks = find_local_peaks(fmap, threshold=adaptive_thr, neighborhood=neighborhood)
    clusters = cluster_peaks(peaks, cluster_radius=cluster_radius)

    detections = []
    img_h, img_w = image_shape
    border_x = int(border_ignore_frac * img_w)
    border_y = int(border_ignore_frac * img_h)

    for idx, (py, px, val) in enumerate(clusters):
        cy, cx = pooled_to_image_coords(py, px, (pooled_h, pooled_w), image_shape)
        # ignore border hits
        if (cx < border_x) or (cx > img_w - border_x) or (cy < border_y) or (cy > img_h - border_y):
            continue

        if fixed_box_size is not None:
            bw, bh = fixed_box_size
        else:
            s = max(0.15, min(1.0, val))
            bw = int(0.15 * img_w * s)
            bh = int(0.25 * img_h * s)

        x1 = max(0, int(cx - bw//2))
        y1 = max(0, int(cy - bh//2))
        w = min(img_w - x1, bw)
        h = min(img_h - y1, bh)
        if w * h < min_box_area:
            continue

        # Assign class label (stub: cycle through CLASSES)
        cls = CLASSES[idx % len(CLASSES)]

        detections.append({
            "x": x1,
            "y": y1,
            "w": w,
            "h": h,
            "confidence": float(round(val, 3)),
            "class": cls
        })

    # keep top-K by confidence
    detections = sorted(detections, key=lambda d: d["confidence"], reverse=True)[:top_k]
    return detections