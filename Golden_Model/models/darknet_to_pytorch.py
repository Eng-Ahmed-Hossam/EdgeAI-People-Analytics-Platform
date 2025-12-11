import torch

cfg_path = "yolov3-tiny.cfg"
weights_path = "yolov3-tiny.weights"
output_path = "yolov3-tiny.pth"

# Load YOLOv3-tiny model from cfg
model = Model(cfg_path)

# Load Darknet weights
model.load_darknet_weights(weights_path)

# Save PyTorch state_dict for later export
torch.save(model.state_dict(), output_path)

print("Saved PyTorch model:", output_path)
