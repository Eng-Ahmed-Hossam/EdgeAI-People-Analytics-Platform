# test_detection_pipeline.py
import cv2
import numpy as np
from collections import deque
import json
import time
import os

from cnn.conv import conv2d_naive
from cnn.relu import relu
from cnn.pooling import max_pooling
from post.detector import detect_from_pooled_improved
from post.nms import non_max_suppression

# ---------- Config ----------
VIDEO_PATH = "people.mov"
FRAME_SIZE = (480, 360)   # (width, height)
POOL_SIZE = 2             # final pooling size (uses this consistently)
POOL_STRIDE = 2
TEMPORAL_LEN = 8          # number of pooled maps to average (increase to smooth more)
MIN_CONF = 0.40
MIN_BOX_AREA = 800
NMS_IOU = 0.25
TOP_K = 8
OUTPUT_JSON = "detections.json"
# ----------------------------

# ensure output JSON exists
if not os.path.exists(OUTPUT_JSON):
    with open(OUTPUT_JSON, "w") as f:
        f.write("")  # create empty file

# read a single frame from the video (for quick test)
cap = cv2.VideoCapture(VIDEO_PATH)
ret, frame = cap.read()
cap.release()
if not ret:
    raise SystemExit(f"Cannot open video: {VIDEO_PATH}")

# resize and prepare grayscale image
frame = cv2.resize(frame, FRAME_SIZE)
img_h, img_w = frame.shape[0], frame.shape[1]
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

# --- convolution chain (pure Python) ---
# first kernel (vertical edge)
kernel1 = np.array([[1, 0, -1],
                    [1, 0, -1],
                    [1, 0, -1]], dtype=np.float32)

conv_out = conv2d_naive(gray, kernel1, stride=1, padding=1)
relu_out = relu(conv_out)

# second kernel (example Laplacian-ish to refine response)
kernel2 = np.array([[0, 1, 0],
                    [1, -4, 1],
                    [0, 1, 0]], dtype=np.float32)

conv2 = conv2d_naive(relu_out, kernel2, stride=1, padding=1)
relu2 = relu(conv2)

# final pooling uses the POOL_SIZE/POOL_STRIDE config
pooled = max_pooling(relu2, pool_size=POOL_SIZE, stride=POOL_STRIDE)  # pooled map

# --- temporal smoothing buffer (for streaming use this across frames) ---
# For single-frame test, this will just contain current frame once,
# but kept here so integration into loop is straightforward.
temporal_buffer = deque(maxlen=TEMPORAL_LEN)
pmin, pmax = float(pooled.min()), float(pooled.max())
if pmax - pmin > 0:
    pooled_norm = (pooled - pmin) / (pmax - pmin)
else:
    pooled_norm = pooled * 0.0
temporal_buffer.append(pooled_norm)
pooled_avg = sum(temporal_buffer) / len(temporal_buffer)

# --- detection using improved detector ---
# image_shape expected as (height, width)
detections = detect_from_pooled_improved(
    pooled_avg,
    image_shape=(img_h, img_w),
    min_conf=MIN_CONF,
    neighborhood=1,
    fixed_box_size=None,
    border_ignore_frac=0.05,
    min_box_area=MIN_BOX_AREA,
    top_k=TOP_K,
    cluster_radius=1
)

# --- apply NMS ---
dets_nms = non_max_suppression(detections, iou_threshold=NMS_IOU)

# --- draw detections on original frame ---
for d in dets_nms:
    x, y, w, h, conf = d["x"], d["y"], d["w"], d["h"], d["confidence"]
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(frame, f"{conf:.2f}", (x, max(12, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

# --- show visual outputs ---
cv2.imshow("Original with Detections", frame)

# show pooled averaged heatmap (normalized to 0..255)
heatmap = (pooled_avg * 255.0).astype("uint8")
# upscale heatmap for easier visualization next to frame
heatmap_resized = cv2.resize(heatmap, FRAME_SIZE)
cv2.imshow("Pooled (smoothed, normalized)", heatmap_resized)

# --- write JSON entry (one timestamp for this run) ---
json_entry = {
    "timestamp": int(time.time()),
    "detections": dets_nms
}
with open(OUTPUT_JSON, "a") as f:
    f.write(json.dumps(json_entry, indent=4))
    f.write(",\n")

print("Detections (after improved detector & NMS):")
print(json.dumps(dets_nms, indent=2))
print(f"Saved JSON entry to: {OUTPUT_JSON}")
print("Press any key in an image window to exit.")
cv2.waitKey(0)
cv2.destroyAllWindows()
