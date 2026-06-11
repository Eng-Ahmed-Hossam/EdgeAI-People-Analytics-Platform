import cv2
import numpy as np
from cnn.conv import conv2d_naive, run_conv_layers
from cnn.relu import relu
from cnn.pooling import max_pooling, run_pooling_layers

# choose input source: either 'video' or 'camera'
SOURCE = "video"
VIDEO_PATH = "people.mov"   # change as needed
CAM_INDEX = 0

# small kernel (vertical edge detector) to match earlier tests
kernel = np.array([[1,0,-1],
                   [1,0,-1],
                   [1,0,-1]], dtype=np.float32)

# read one frame
if SOURCE == "video":
    cap = cv2.VideoCapture(VIDEO_PATH)
else:
    cap = cv2.VideoCapture(CAM_INDEX)

ret, frame = cap.read()
cap.release()
if not ret:
    raise SystemExit("Cannot read frame from source")

# resize for speed (match your pipeline)
FRAME_SIZE = (160, 120)   # small so pure Python is OK
frame = cv2.resize(frame, FRAME_SIZE)

# convert to grayscale (single channel, add channel dim)
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
gray = np.expand_dims(gray, axis=-1)  # H,W,1

# --- conv using run_conv_layers ---
layer_cfg = {
    "kernel": kernel[..., None],  # expand to (3,3,1)
    "bias": 0.0,
    "stride": 1,
    "padding": 1
}
conv_out, mem_usage = run_conv_layers(gray, [layer_cfg], apply_relu=False)

# --- relu ---
relu_out = relu(conv_out)
if relu_out.ndim == 2:  # ensure channel dimension
    relu_out = np.expand_dims(relu_out, axis=-1)

# --- pooling ---
pooled, _ = run_pooling_layers(relu_out[:, :, 0], [{"pool_size": 2, "stride": 2}])

# normalize for display (0..255)
def norm_for_display(x):
    xm = x - x.min()
    if xm.max() > 0:
        xm = xm / xm.max() * 255.0
    return xm.astype(np.uint8)

cv2.imshow("Original", frame)
cv2.imshow("Conv (heatmap)", norm_for_display(conv_out[:, :, 0]))
cv2.imshow("ReLU (heatmap)", norm_for_display(relu_out[:, :, 0]))
cv2.imshow("Pooled (heatmap)", norm_for_display(pooled))

print("Shapes -> conv:", conv_out.shape, "relu:", relu_out.shape, "pooled:", pooled.shape)
print("Memory usage (bytes):", mem_usage)
print("Press any key in window to exit")
cv2.waitKey(0)
cv2.destroyAllWindows()