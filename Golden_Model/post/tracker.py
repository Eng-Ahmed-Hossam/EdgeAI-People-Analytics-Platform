# post/tracker.py
import time
from typing import List, Dict, Tuple
import numpy as np

def bbox_center(box: Dict) -> Tuple[int,int]:
    """Return center (cx, cy) of box {x,y,w,h}"""
    cx = int(box["x"] + box["w"] / 2)
    cy = int(box["y"] + box["h"] / 2)
    return (cx, cy)

class CentroidTracker:
    """
    Simple centroid tracker:
    - Register new object with incremental ID
    - Match new detections by Euclidean distance to existing centroids
    - Deregister objects that disappear for too long (max_disappeared frames)
    - Track first_seen, last_seen timestamps for dwell time
    """
    def __init__(self, max_disappeared=30, max_distance=80):
        self.next_object_id = 1
        self.objects = {}  # id -> box dict (x,y,w,h,confidence)
        self.centroids = {}  # id -> (cx,cy)
        self.disappeared = {}  # id -> consecutive missed frames
        self.first_seen = {}   # id -> first seen timestamp (seconds)
        self.last_seen = {}    # id -> last seen timestamp (seconds)
        self.max_disappeared = max_disappeared
        self.max_distance = max_distance

    def register(self, box: Dict, now_ts: float):
        oid = self.next_object_id
        self.next_object_id += 1
        self.objects[oid] = box.copy()
        self.centroids[oid] = bbox_center(box)
        self.disappeared[oid] = 0
        self.first_seen[oid] = now_ts
        self.last_seen[oid] = now_ts
        return oid

    def deregister(self, oid: int):
        if oid in self.objects:
            del self.objects[oid]
            del self.centroids[oid]
            del self.disappeared[oid]
            del self.first_seen[oid]
            del self.last_seen[oid]

    def update(self, detections: List[Dict], now_ts: float = None) -> List[Dict]:
        """
        detections: list of dicts with x,y,w,h,confidence (no id)
        Returns list of tracked dicts with fields:
        {id, x,y,w,h, confidence, dwell_time}
        """
        if now_ts is None:
            now_ts = time.time()

        # If no detections, mark all existing as disappeared and deregister if needed
        if len(detections) == 0:
            remove = []
            for oid in list(self.disappeared.keys()):
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    remove.append(oid)
            for oid in remove:
                self.deregister(oid)
            # return current tracked (none may remain)
            return self._tracked_list(now_ts)

        # compute input centroids
        input_centroids = []
        for d in detections:
            cx = int(d["x"] + d["w"] / 2)
            cy = int(d["y"] + d["h"] / 2)
            input_centroids.append((cx, cy))

        # if no existing objects, register all
        if len(self.centroids) == 0:
            for d in detections:
                oid = self.register(d, now_ts)
                self.objects[oid] = d.copy()
            return self._tracked_list(now_ts)

        # Build arrays for distance computation
        object_ids = list(self.centroids.keys())
        object_centroids = [self.centroids[oid] for oid in object_ids]

        D = np.zeros((len(object_centroids), len(input_centroids)), dtype=float)
        for i, oc in enumerate(object_centroids):
            for j, ic in enumerate(input_centroids):
                D[i, j] = np.linalg.norm(np.array(oc) - np.array(ic))

        # greedy match: smallest distances first
        rows = D.min(axis=1).argsort()  # order existing objects by min distance
        cols = D.argmin(axis=1)[rows]

        assigned_rows = set()
        assigned_cols = set()

        for r, c in zip(rows, cols):
            if r in assigned_rows or c in assigned_cols:
                continue
            if D[r, c] > self.max_distance:
                continue  # too far, skip match
            oid = object_ids[r]
            # assign detection c to object oid
            self.disappeared[oid] = 0
            self.objects[oid] = detections[c].copy()
            self.centroids[oid] = input_centroids[c]
            self.last_seen[oid] = now_ts
            assigned_rows.add(r)
            assigned_cols.add(c)

        # any unassigned existing objects -> increase disappeared counter
        for i, oid in enumerate(object_ids):
            if i not in assigned_rows:
                self.disappeared[oid] += 1
                if self.disappeared[oid] > self.max_disappeared:
                    self.deregister(oid)

        # any unassigned new detections -> register
        for j, d in enumerate(detections):
            if j not in assigned_cols:
                self.register(d, now_ts)

        return self._tracked_list(now_ts)

    def _tracked_list(self, now_ts: float):
        tracked = []
        for oid in sorted(self.objects.keys()):
            box = self.objects[oid]
            dwell = float(round(now_ts - self.first_seen[oid], 2)) if oid in self.first_seen else 0.0
            tracked.append({
                "id": int(oid),
                "x": int(box["x"]),
                "y": int(box["y"]),
                "w": int(box["w"]),
                "h": int(box["h"]),
                "confidence": float(box.get("confidence", 0.0)),
                "dwell_time": dwell
            })
        return tracked
