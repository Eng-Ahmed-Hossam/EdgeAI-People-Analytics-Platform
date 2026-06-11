# cnn/conv.py
"""
Pure-Python convolution implementations to mirror FPGA sliding-window + MAC.
- conv2d_naive: obvious nested-loop implementation (single-channel)
- conv2d_multi_channel: sums over input channels (implements standard Conv2D)
- conv2d_numpy: vectorized/numpy version used as reference for correctness
- run_conv_layers: helper to apply multiple conv layers sequentially
"""

import numpy as np
from typing import Tuple, List
from cnn.quant import quantize_tensor, dequantize_tensor, QuantParams
from cnn.relu import relu

def conv2d_naive(image: np.ndarray, kernel: np.ndarray, stride: int = 1,
                 padding: int = 0, qparams: QuantParams=None) -> np.ndarray:
    """Single-channel convolution (valid with optional zero padding)."""
    if padding > 0:
        img = np.pad(image, ((padding, padding), (padding, padding)),
                     mode='constant', constant_values=0)
    else:
        img = image

    H, W = img.shape
    kH, kW = kernel.shape
    out_h = (H - kH) // stride + 1
    out_w = (W - kW) // stride + 1
    out = np.zeros((out_h, out_w), dtype=np.float32)

    # Sliding window + MAC
    for oy in range(out_h):
        for ox in range(out_w):
            acc = 0.0
            y0 = oy * stride
            x0 = ox * stride
            for ky in range(kH):
                for kx in range(kW):
                    pix = float(img[y0 + ky, x0 + kx])
                    w = float(kernel[ky, kx])
                    acc += pix * w
            out[oy, ox] = acc

    if qparams:
        return quantize_tensor(out, qparams)
    return out

def conv2d_multi_channel(image: np.ndarray, kernels: np.ndarray, bias: float = 0.0,
                         stride: int = 1, padding: int = 0) -> np.ndarray:
    """Multi-channel convolution producing a single output channel."""
    if image.ndim != 3:
        raise ValueError("image must be H x W x C")

    H, W, C = image.shape
    kH, kW, kC = kernels.shape
    assert kC == C, "Kernel channels must match image channels"

    img = image.astype(np.float32)
    ker = kernels.astype(np.float32)

    if padding > 0:
        img = np.pad(img, ((padding, padding), (padding, padding), (0, 0)),
                     mode='constant', constant_values=0)

    H2, W2 = img.shape[0], img.shape[1]
    out_h = (H2 - kH) // stride + 1
    out_w = (W2 - kW) // stride + 1
    out = np.zeros((out_h, out_w), dtype=np.float32)

    for oy in range(out_h):
        for ox in range(out_w):
            acc = 0.0
            y0 = oy * stride
            x0 = ox * stride
            for ky in range(kH):
                for kx in range(kW):
                    for ch in range(C):
                        pix = float(img[y0 + ky, x0 + kx, ch])
                        w = float(ker[ky, kx, ch])
                        acc += pix * w
            out[oy, ox] = acc + float(bias)

    # Ensure output has a channel dimension (H, W, 1)
    return np.expand_dims(out, axis=-1)

def conv2d_numpy(image: np.ndarray, kernel: np.ndarray, stride: int = 1,
                 padding: int = 0) -> np.ndarray:
    """Numpy-based reference for single-channel convolution."""
    if padding > 0:
        img = np.pad(image, ((padding, padding), (padding, padding)),
                     mode='constant', constant_values=0)
    else:
        img = image

    H, W = img.shape
    kH, kW = kernel.shape
    out_h = (H - kH) // stride + 1
    out_w = (W - kW) // stride + 1
    out = np.zeros((out_h, out_w), dtype=np.float32)

    for oy in range(out_h):
        for ox in range(out_w):
            y0 = oy * stride
            x0 = ox * stride
            patch = img[y0:y0+kH, x0:x0+kW]
            out[oy, ox] = np.sum(patch * kernel)

    return out

def run_conv_layers(input_map: np.ndarray,
                    conv_layers: List[dict],
                    apply_relu: bool = True) -> Tuple[np.ndarray, int]:
    """
    Run a sequence of conv layers with optional ReLU.
    conv_layers: list of dicts, each with {"kernel": np.ndarray, "bias": float, "stride": int, "padding": int}
    Returns (final feature map, memory_usage_bytes)
    """
    fmap = input_map
    # Ensure input has channel dimension
    if fmap.ndim == 2:
        fmap = np.expand_dims(fmap, axis=-1)

    memory_usage = 0
    for layer in conv_layers:
        fmap = conv2d_multi_channel(fmap,
                                    kernels=layer["kernel"],
                                    bias=layer.get("bias", 0.0),
                                    stride=layer.get("stride", 1),
                                    padding=layer.get("padding", 0))
        if apply_relu:
            fmap = relu(fmap)
            # relu may return 2D, so re‑expand
            if fmap.ndim == 2:
                fmap = np.expand_dims(fmap, axis=-1)

        # track memory usage (weights + activations)
        memory_usage += layer["kernel"].size * 4  # float32 bytes
        memory_usage += fmap.size * 4
    return fmap, memory_usage