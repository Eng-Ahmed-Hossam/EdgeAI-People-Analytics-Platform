import cv2
import numpy as np

# Paths to YOLO-tiny model files
CFG_FILE = "models/yolov3-tiny.cfg"
WEIGHTS_FILE = "models/yolov3-tiny.weights"
NAMES_FILE = "models/coco.names"

with open(NAMES_FILE, "r") as f:
    CLASSES = [line.strip() for line in f.readlines()]

net = cv2.dnn.readNetFromDarknet(CFG_FILE, WEIGHTS_FILE)
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

def run_yolo(frame, conf_threshold=0.4, nms_threshold=0.3):
    (h, w) = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416),
                                 swapRB=True, crop=False)
    net.setInput(blob)
    layer_outputs = net.forward(output_layers)

    boxes, confidences, classIDs = [], [], []
    for output in layer_outputs:
        for detection in output:
            scores = detection[5:]
            classID = int(scores.argmax())
            confidence = scores[classID]
            if confidence > conf_threshold:
                box = detection[0:4] * [w, h, w, h]
                (centerX, centerY, width, height) = box.astype("int")
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
                classIDs.append(classID)

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    results = []
    if len(idxs) > 0:
        for i in idxs.flatten():
            label = CLASSES[classIDs[i]]
            if label in ["person", "car", "motorbike"]:
                results.append({
                    "x": boxes[i][0],
                    "y": boxes[i][1],
                    "w": boxes[i][2],
                    "h": boxes[i][3],
                    "confidence": confidences[i],
                    "class": "motorcycle" if label == "motorbike" else label
                })
    return results

# --- NEW: layer loader ---
def load_tiny_yolo_layers(n: int):
    """
    Extract the first n conv layers from YOLO-tiny.
    Returns a list of dicts with kernel, bias, stride, padding.
    """
    layers = []
    conv_count = 0

    for name in layer_names:
        if "conv" in name.lower():
            lid = net.getLayerId(name)
            try:
                w = net.getParam(lid, 0)  # weights: out_c, in_c, kH, kW
                b = net.getParam(lid, 1)  # biases
            except cv2.error:
                continue

            w = np.array(w)
            b = np.array(b).flatten()

            # For demo: take the first output channel
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