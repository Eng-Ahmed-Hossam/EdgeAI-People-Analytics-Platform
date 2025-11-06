import tkinter as tk
from tkinter import filedialog
import threading
import cv2
import time
import json
import numpy as np
from collections import deque

from core.frame_reader import FrameReader
from utils.config import FRAME_SIZE

# --- Model wrappers ---
from cnn.tiny_yolo import run_yolo, load_tiny_yolo_layers
from cnn.mobilenet import run_mobilenet, load_mobilenet_layers
from cnn.conv import run_conv_layers
from cnn.relu import relu
from cnn.pooling import run_pooling_layers
from post.detector import detect_from_pooled_improved
from post.nms import non_max_suppression

OUTPUT_JSON = "detections.jsonl"
running = False
selected_model = None   # "yolo", "mobilenet", "mycnn"
num_layers = 1          # default number of conv layers

# --- Config ---
POOL_SIZE = 2
POOL_STRIDE = 2
TEMPORAL_LEN = 8
MIN_CONF = 0.40
MIN_BOX_AREA = 800
NMS_IOU = 0.25
TOP_K = 8

temporal_buffer = deque(maxlen=TEMPORAL_LEN)

def run_mycnn(frame):
    """Golden model pipeline using N conv layers from chosen pretrained model."""
    img_h, img_w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    gray = np.expand_dims(gray, axis=-1)  # H,W,1

    # --- load conv layers depending on selected_model ---
    if selected_model == "mycnn":
        # for demo: just use toy kernels
        conv_layers = [{"kernel": np.ones((3,3,1),dtype=np.float32),
                        "bias": 0.0, "stride":1, "padding":1}] * num_layers
    elif selected_model == "yolo":
        conv_layers = load_tiny_yolo_layers(num_layers)
    elif selected_model == "mobilenet":
        conv_layers = load_mobilenet_layers(num_layers)
    else:
        conv_layers = []

    # --- run conv layers ---
    fmap, _ = run_conv_layers(gray, conv_layers, apply_relu=True)

    # --- pooling ---
    pooled, _ = run_pooling_layers(fmap[:,:,0], [{"pool_size": POOL_SIZE, "stride": POOL_STRIDE}])

    # normalize + temporal smoothing
    pmin, pmax = float(pooled.min()), float(pooled.max())
    pooled_norm = (pooled - pmin)/(pmax-pmin) if pmax>pmin else pooled*0.0
    temporal_buffer.append(pooled_norm)
    pooled_avg = sum(temporal_buffer)/len(temporal_buffer)

    # detection + NMS
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
    return non_max_suppression(detections, iou_threshold=NMS_IOU)

def run_pipeline(source, is_camera=False):
    global running, selected_model
    running = True
    fr = FrameReader(video_path=None if is_camera else source,
                     camera_index=0 if is_camera else None)

    with open(OUTPUT_JSON, "w") as f:
        pass

    frame_id = 0
    while running:
        ret, frame = fr.read()
        if not ret:
            break
        frame_id += 1
        frame_resized = cv2.resize(frame, FRAME_SIZE)

        # --- choose model ---
        if selected_model == "yolo":
            detections = run_yolo(frame_resized)
        elif selected_model == "mobilenet":
            detections = run_mobilenet(frame_resized)
        elif selected_model == "mycnn":
            detections = run_mycnn(frame_resized)
        else:
            detections = []

        # draw boxes
        for d in detections:
            x,y,w,h,conf = d["x"],d["y"],d["w"],d["h"],d["confidence"]
            cls = d.get("class","object")
            color = (0,255,0) if cls=="person" else (255,0,0) if cls=="car" else (0,0,255)
            cv2.rectangle(frame_resized,(x,y),(x+w,y+h),color,2)
            cv2.putText(frame_resized,f"{cls} {conf:.2f}",(x,max(12,y-6)),
                        cv2.FONT_HERSHEY_SIMPLEX,0.6,color,2)

        entry = {"timestamp": int(time.time()), "frame_id": frame_id,
                 "model": selected_model, "layers": num_layers, "detections": detections}
        with open(OUTPUT_JSON, "a") as f:
            f.write(json.dumps(entry) + "\n")

        cv2.imshow("Detections", frame_resized)
        if cv2.waitKey(1) & 0xFF == 27:
            running = False
            break

    fr.release()
    cv2.destroyAllWindows()

def open_video():
    path = filedialog.askopenfilename(filetypes=[("Video files", "*.mp4 *.avi *.mov")])
    if path:
        threading.Thread(target=run_pipeline, args=(path, False), daemon=True).start()

def open_camera():
    threading.Thread(target=run_pipeline, args=(None, True), daemon=True).start()

def stop_pipeline():
    global running
    running = False

def on_closing():
    stop_pipeline()
    root.destroy()
    cv2.destroyAllWindows()

# --- GUI ---
root = tk.Tk()
root.title("Edge CNN App")

# Model selection
def select_yolo():
    global selected_model
    selected_model = "yolo"
    lbl_model.config(text=f"Selected: YOLO-tiny ({num_layers} layers)")

def select_mobilenet():
    global selected_model
    selected_model = "mobilenet"
    lbl_model.config(text=f"Selected: MobileNet-SSD ({num_layers} layers)")

def select_mycnn():
    global selected_model
    selected_model = "mycnn"
    lbl_model.config(text=f"Selected: My CNN ({num_layers} layers)")

btn_yolo = tk.Button(root, text="Use YOLO-tiny", command=select_yolo, width=20, height=2)
btn_yolo.pack(pady=5)

btn_mobilenet = tk.Button(root, text="Use MobileNet-SSD", command=select_mobilenet, width=20, height=2)
btn_mobilenet.pack(pady=5)

btn_mycnn = tk.Button(root, text="Use My CNN", command=select_mycnn, width=20, height=2)
btn_mycnn.pack(pady=5)

lbl_model = tk.Label(root, text="No model selected")
lbl_model.pack(pady=5)

# --- Layer control ---
def set_layers(val):
    global num_layers
    num_layers = int(val)
    if selected_model:
        lbl_model.config(text=f"Selected: {selected_model} ({num_layers} layers)")

layer_scale = tk.Scale(root, from_=1, to=10, orient=tk.HORIZONTAL,
                       label="Number of Conv Layers", command=set_layers)
layer_scale.set(1)
layer_scale.pack(pady=10)

btn_video = tk.Button(root, text="Open Video", command=open_video, width=20, height=2)
btn_video.pack(pady=10)

btn_camera = tk.Button(root, text="Open Camera", command=open_camera, width=20, height=2)
btn_camera.pack(pady=10)

btn_stop = tk.Button(root, text="Stop", command=stop_pipeline, width=20, height=2)
btn_stop.pack(pady=10)

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()