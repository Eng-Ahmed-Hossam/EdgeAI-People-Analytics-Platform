import cv2
import numpy as np

# Paths to pretrained MobileNet-SSD model files
PROTO_TXT = "models/MobileNetSSD_deploy.prototxt"
MODEL_FILE = "models/MobileNetSSD_deploy.caffemodel"

# COCO/VOC class labels for MobileNet-SSD
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant",
           "sheep", "sofa", "train", "tvmonitor"]

net = cv2.dnn.readNetFromCaffe(PROTO_TXT, MODEL_FILE)

def run_mobilenet(frame, conf_threshold=0.4):
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                                 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    results = []
    for i in range(detections.shape[2]):
        confidence = detections[0, 0, i, 2]
        if confidence > conf_threshold:
            idx = int(detections[0, 0, i, 1])
            label = CLASSES[idx]
            if label in ["person", "car", "motorbike"]:
                box = detections[0, 0, i, 3:7] * [w, h, w, h]
                (startX, startY, endX, endY) = box.astype("int")
                results.append({
                    "x": int(startX),
                    "y": int(startY),
                    "w": int(endX - startX),
                    "h": int(endY - startY),
                    "confidence": float(confidence),
                    "class": "motorcycle" if label == "motorbike" else label
                })
    return results

# --- NEW: layer loader ---
def load_mobilenet_layers(n: int):
    """
    Extract the first n conv layers from MobileNet-SSD.
    Returns a list of dicts with kernel, bias, stride, padding.
    """
    layers = []
    layer_names = net.getLayerNames()
    conv_count = 0

    for name in layer_names:
        if "conv" in name.lower():  # crude filter for conv layers
            lid = net.getLayerId(name)
            try:
                w = net.getParam(lid, 0)  # weights: out_c, in_c, kH, kW
                b = net.getParam(lid, 1)  # biases
            except cv2.error:
                continue

            w = np.array(w)  # convert to numpy
            b = np.array(b).flatten()

            # For demo: take first output channel
            kernel = np.transpose(w[0], (1, 2, 0))  # (kH, kW, in_c)
            bias = float(b[0]) if b.size > 0 else 0.0

            layers.append({
                "kernel": kernel.astype(np.float32),
                "bias": bias,
                "stride": 1,
                "padding": 1
            })

            conv_count += 1
            if conv_count >= n:
                break

    return layers