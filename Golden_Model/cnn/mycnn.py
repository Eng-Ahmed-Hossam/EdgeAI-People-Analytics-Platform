import numpy as np
import cv2
from collections import deque
from cnn.conv import conv2d_naive
from cnn.relu import relu
from cnn.pooling import max_pooling
from post.detector import detect_from_pooled_improved
from post.nms import non_max_suppression

# Config
POOL_SIZE = 2
POOL_STRIDE = 2
TEMPORAL_LEN = 8
MIN_CONF = 0.40
MIN_BOX_AREA = 800
NMS_IOU = 0.25
TOP_K = 8

temporal_buffer = deque(maxlen=TEMPORAL_LEN)

# Define your label set
CLASSES = ["person", "car", "motorcycle"]

def run_mycnn(frame):
    img_h, img_w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)

    # --- conv → relu → conv → relu → pool ---
    k1 = np.array([[1,0,-1],
                   [1,0,-1],
                   [1,0,-1]], dtype=np.float32)
    conv1 = conv2d_naive(gray, k1, stride=1, padding=1)
    relu1 = relu(conv1)

    k2 = np.array([[0,1,0],
                   [1,-4,1],
                   [0,1,0]], dtype=np.float32)
    conv2 = conv2d_naive(relu1, k2, stride=1, padding=1)
    relu2 = relu(conv2)

    pooled = max_pooling(relu2, pool_size=POOL_SIZE, stride=POOL_STRIDE)

    # --- normalize + temporal smoothing ---
    pmin, pmax = float(pooled.min()), float(pooled.max())
    pooled_norm = (pooled - pmin)/(pmax - pmin) if pmax > pmin else pooled * 0.0
    temporal_buffer.append(pooled_norm)
    pooled_avg = sum(temporal_buffer) / len(temporal_buffer)

    # --- detection ---
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

    # --- assign classes (stub logic) ---
    for i, det in enumerate(detections):
        det["class"] = CLASSES[i % len(CLASSES)]
        # Alternative: det["class"] = np.random.choice(CLASSES)

    # --- NMS ---
    return non_max_suppression(detections, iou_threshold=NMS_IOU)