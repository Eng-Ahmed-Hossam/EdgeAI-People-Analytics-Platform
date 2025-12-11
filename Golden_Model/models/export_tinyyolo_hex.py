#!/usr/bin/env python3
"""
export_tinyyolo_hex.py

Standalone script: parses TinyYOLOv3 from .cfg + .weights,
folds BN, quantizes conv layers to INT8, bias to INT32,
exports HEX + Numpy + metadata, including pooling info.
"""

import os
import argparse
import json
import numpy as np

# ----------------- CLI -----------------
parser = argparse.ArgumentParser()
parser.add_argument("--cfg", type=str, required=True, help="TinyYOLO .cfg file")
parser.add_argument("--weights", type=str, required=True, help="TinyYOLO .weights file")
parser.add_argument("--out", type=str, default="yolo_hex", help="Output directory")
parser.add_argument("--input_scale", type=float, default=1.0, help="Activation input scale for bias")
parser.add_argument("--save_float", action="store_true", help="Save float32 folded weights/biases")
parser.add_argument("--per_channel", action="store_true", help="Per-output-channel quantization")
args = parser.parse_args()

OUTPUT_DIR = args.out
os.makedirs(OUTPUT_DIR, exist_ok=True)
INPUT_SCALE = args.input_scale
SAVE_FLOAT = args.save_float
PER_CHANNEL = True if args.per_channel else False

# ----------------- Darknet CFG Parser -----------------
def parse_cfg(cfg_file):
    """Parse Darknet cfg and return list of blocks"""
    with open(cfg_file, "r") as f:
        lines = f.read().split("\n")
    lines = [x.strip() for x in lines if x.strip() and not x.strip().startswith("#")]
    blocks = []
    block = {}
    for line in lines:
        if line.startswith("["):
            if block:
                blocks.append(block)
            block = {"type": line[1:-1].strip()}
        else:
            key, val = line.split("=")
            block[key.strip()] = val.strip()
    blocks.append(block)
    return blocks

# ----------------- Helpers: Quantization & HEX -----------------
def quantize_weights_per_channel(W):
    out_ch = W.shape[0]
    W_q = np.zeros_like(W, dtype=np.int8)
    scales = np.zeros(out_ch, dtype=np.float32)
    for c in range(out_ch):
        w_c = W[c]
        max_abs = np.max(np.abs(w_c))
        scale = max_abs / 127.0 if max_abs != 0 else 1.0
        q = np.round(w_c / scale).astype(np.int8)
        q = np.clip(q, -128, 127)
        W_q[c] = q
        scales[c] = scale
    return W_q, scales

def quantize_bias_int32(b, scales, input_scale):
    b_q = np.zeros_like(b, dtype=np.int32)
    for c in range(b.shape[0]):
        denom = scales[c] * input_scale
        b_q[c] = int(np.round(b[c] / denom)) if denom != 0 else 0
    return b_q

def int8_to_hex(arr):
    return ["{:02x}".format(int(np.int8(x)) & 0xFF) for x in arr.flatten()]

def int32_to_hex(arr):
    return ["{:08x}".format(int(np.int32(x)) & 0xFFFFFFFF) for x in arr.flatten()]

def save_layer(prefix, W_float, W_q, scales, b_float, b_q):
    w_float_fname = os.path.join(OUTPUT_DIR, f"{prefix}_conv_weights.npy")
    w_int8_fname = os.path.join(OUTPUT_DIR, f"{prefix}_conv_weights_int8.npy")
    w_hex_fname = os.path.join(OUTPUT_DIR, f"{prefix}_conv_weights.hex")
    b_int32_fname = os.path.join(OUTPUT_DIR, f"{prefix}_conv_bias_int32.npy")
    b_hex_fname = os.path.join(OUTPUT_DIR, f"{prefix}_conv_bias.hex")
    meta_fname = os.path.join(OUTPUT_DIR, f"{prefix}_meta.json")

    if SAVE_FLOAT and W_float is not None:
        np.save(w_float_fname, W_float.astype(np.float32))
    np.save(w_int8_fname, W_q)
    np.save(b_int32_fname, b_q)

    with open(w_hex_fname, "w") as f:
        f.write("\n".join(int8_to_hex(W_q)))
    with open(b_hex_fname, "w") as f:
        f.write("\n".join(int32_to_hex(b_q)))

    meta = {
        "weight_shape": list(W_q.shape),
        "weight_dtype": "int8",
        "weight_scales": scales.tolist(),
        "bias_shape": list(b_q.shape),
        "bias_dtype": "int32",
        "activation_input_scale_used": INPUT_SCALE,
        "weight_hex_file": os.path.basename(w_hex_fname),
        "bias_hex_file": os.path.basename(b_hex_fname)
    }
    with open(meta_fname, "w") as f:
        json.dump(meta, f, indent=2)
    print(f"Saved layer {prefix}")

# ----------------- Load Darknet Weights -----------------
def load_darknet_weights(weights_file, blocks):
    fp = open(weights_file, "rb")
    header = np.fromfile(fp, dtype=np.int32, count=5)
    weights = np.fromfile(fp, dtype=np.float32)
    ptr = 0
    layers_data = []

    in_c = 3  # initial input channels
    for block in blocks:
        if block["type"] == "convolutional":
            filters = int(block["filters"])
            k = int(block["size"])
            batch_normalize = int(block.get("batch_normalize", 0))
            conv_shape = (filters, in_c, k, k)

            if batch_normalize:
                # BN order: bias, gamma, mean, var
                bn_bias = weights[ptr:ptr+filters]; ptr += filters
                bn_gamma = weights[ptr:ptr+filters]; ptr += filters
                bn_mean = weights[ptr:ptr+filters]; ptr += filters
                bn_var = weights[ptr:ptr+filters]; ptr += filters
                num_w = np.prod(conv_shape)
                conv_W = weights[ptr:ptr+num_w].reshape(conv_shape); ptr += num_w
                layers_data.append({
                    "type":"conv_bn",
                    "W": conv_W,
                    "bn_bias": bn_bias,
                    "bn_gamma": bn_gamma,
                    "bn_mean": bn_mean,
                    "bn_var": bn_var
                })
            else:
                bias = weights[ptr:ptr+filters]; ptr += filters
                num_w = np.prod(conv_shape)
                conv_W = weights[ptr:ptr+num_w].reshape(conv_shape); ptr += num_w
                layers_data.append({
                    "type":"conv",
                    "W": conv_W,
                    "bias": bias
                })

            in_c = filters  # update input channels

        elif block["type"] in ["maxpool","avgpool"]:
            layers_data.append({
                "type": block["type"],
                "kernel_size": int(block.get("size",2)),
                "stride": int(block.get("stride",2)),
                "padding": int(block.get("pad",0))
            })

    fp.close()
    return layers_data

# ----------------- Process & Export -----------------
blocks = parse_cfg(args.cfg)
layers_data = load_darknet_weights(args.weights, blocks)

layer_idx = 0
for layer in layers_data:
    if layer["type"] in ["conv","conv_bn"]:
        W_float = None
        b_float = None
        if layer["type"] == "conv_bn":
            gamma = layer["bn_gamma"]
            beta = layer["bn_bias"]
            mean = layer["bn_mean"]
            var = layer["bn_var"]
            eps = 1e-5
            std = np.sqrt(np.maximum(var, 0.0) + eps)  # avoid NaN
            scale = gamma / std
            W_float = layer["W"] * scale.reshape(-1,1,1,1)
            b_float = (-mean) * scale + beta
        else:
            W_float = layer["W"]
            b_float = layer["bias"]
        W_q, scales = quantize_weights_per_channel(W_float)
        b_q = quantize_bias_int32(b_float, scales, INPUT_SCALE)
        save_layer(f"layer{layer_idx}", W_float, W_q, scales, b_float, b_q)
        layer_idx +=1
    elif layer["type"] in ["maxpool","avgpool"]:
        info = {
            "type": "max" if layer["type"]=="maxpool" else "avg",
            "kernel_size": layer["kernel_size"],
            "stride": layer["stride"],
            "padding": layer["padding"]
        }
        fname = os.path.join(OUTPUT_DIR, f"layer{layer_idx}_pooling.json")
        with open(fname,"w") as f:
            json.dump(info,f,indent=2)
        print(f"Saved pooling layer {layer_idx}")
        layer_idx +=1

print(f"[INFO] Done. Exported {layer_idx} layers to {OUTPUT_DIR}")
