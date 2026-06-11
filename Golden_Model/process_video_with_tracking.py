# process_video_with_tracking.py
import cv2
import numpy as np
import time
import json
from collections import deque
import os

# Optional: MQTT (install paho-mqtt if you want MQTT publish)
try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except Exception:
    MQTT_AVAILABLE = False

from cnn.conv import conv2d_naive
from cnn.relu import relu
from cnn.pooling import max_pooling
from post.detector import detect_from_pooled_improved
from post.nms import non_max_suppression
from post.tracker import CentroidTracker

# ---------- CONFIG ----------
VIDEO_PATH = "people.mov"          # or use 0 for webcam
USE_CAMERA = False                 # set True to use camera index 0
CAM_INDEX = 0

FRAME_SIZE = (480, 360)            # width, height
POOL_SIZE = 4
POOL_STRIDE = 4
TEMPORAL_LEN = 5                   # smoothing window length
MIN_CONF = 0.30
NMS_IOU = 0.30
TOP_K = 12
MIN_BOX_AREA = 400

JSON_PATH = "detections.json"
MQTT_ENABLED = False               # set True to enable MQTT publish
MQTT_BROKER = "broker.hivemq.com"
MQTT_TOPIC = "fpga/edge/detections"
# -----------------------------

# Prepare MQTT client if enabled
mqtt_client = None
if MQTT_ENABLED:
    if not MQTT_AVAILABLE:
        raise RuntimeError("MQTT not available. Install paho-mqtt.")
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, 1883, 60)
    mqtt_client.loop_start()

# tracker
tracker = CentroidTracker(max_disappeared=30, max_distance=80)

# temporal buffer for pooled maps
temporal_buffer = deque(maxlen=TEMPORAL_LEN)

# video source
if USE_CAMERA:
    cap = cv2.VideoCapture(CAM_INDEX)
else:
    cap = cv2.VideoCapture(VIDEO_PATH)
if not cap.isOpened():
    raise SystemExit("Cannot open video/camera.")

# output writer (optional)
save_output = False
writer = None
if save_output:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    writer = cv2.VideoWriter("processed_output.mp4", fourcc, fps, FRAME_SIZE)

# conv kernel (example; later you can replace with learned kernels)
kernel = np.array([[1, 0, -1],
                   [1, 0, -1],
                   [1, 0, -1]], dtype=np.float32)

# JSON batching variables
json_bucket = []            # collects tracked objects within current second
last_json_write = time.time()

frame_id = 0
start_time = time.time()

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[INFO] End of stream.")
            break
        frame_id += 1

        # resize and gray
        frame = cv2.resize(frame, FRAME_SIZE)
        img_h, img_w = frame.shape[0], frame.shape[1]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

        # --- CNN pipeline (pure Python conv/relu/pool) ---
        conv_out = conv2d_naive(gray, kernel, stride=1, padding=1)
        relu_out = relu(conv_out)
        pooled = max_pooling(relu_out, pool_size=POOL_SIZE, stride=POOL_STRIDE)

        # normalize pooled and temporal smoothing
        pmin, pmax = float(pooled.min()), float(pooled.max())
        if pmax - pmin > 0:
            pooled_norm = (pooled - pmin) / (pmax - pmin)
        else:
            pooled_norm = pooled * 0.0
        temporal_buffer.append(pooled_norm)
        pooled_avg = sum(temporal_buffer) / len(temporal_buffer)

        # detect & nms
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
        dets_nms = non_max_suppression(detections, iou_threshold=NMS_IOU)

        # update tracker (gives persistent IDs + dwell)
        now_ts = time.time()
        tracked = tracker.update(dets_nms, now_ts)

        # draw tracked boxes
        for t in tracked:
            x, y, w, h = t["x"], t["y"], t["w"], t["h"]
            tid = t["id"]
            conf = t["confidence"]
            dwell = t["dwell_time"]
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{tid} {dwell:.1f}s", (x, max(12, y - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # add tracked objects to JSON bucket for this second
        for t in tracked:
            json_bucket.append({
                "id": int(t["id"]),
                "class": "object",   # placeholder; your detector can add class later
                "x": int(t["x"]),
                "y": int(t["y"]),
                "w": int(t["w"]),
                "h": int(t["h"]),
                "confidence": float(t["confidence"]),
                "dwell_time": float(t["dwell_time"])
            })

        # every 1 second -> write one JSON entry and optionally publish
        if now_ts - last_json_write >= 1.0:
            entry = {
                "timestamp": int(last_json_write),
                "objects": json_bucket.copy()
            }
            with open(JSON_PATH, "a") as f:
                f.write(json.dumps(entry, indent=4))
                f.write(",\n")

            if MQTT_ENABLED and mqtt_client is not None:
                mqtt_client.publish(MQTT_TOPIC, json.dumps(entry))

            json_bucket.clear()
            last_json_write = now_ts

        # show and write output
        cv2.imshow("Tracked Output", frame)
        if writer:
            writer.write(frame)

        # exit keys
        if cv2.waitKey(1) & 0xFF == 27:
            print("[INFO] ESC pressed. Exiting.")
            break

finally:
    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    if MQTT_ENABLED and mqtt_client:
        mqtt_client.loop_stop()
    print(f"[INFO] Processed frames: {frame_id}. Elapsed: {time.time()-start_time:.1f}s")
