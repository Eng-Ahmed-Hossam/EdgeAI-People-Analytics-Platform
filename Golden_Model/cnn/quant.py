# cnn/quant.py
import numpy as np
import json

class QuantParams:
    def __init__(self, scale: float, zero_point: int, dtype=np.int8):
        self.scale = float(scale)
        self.zero_point = int(zero_point)
        self.dtype = dtype

def quantize_tensor(x: np.ndarray, qparams: QuantParams) -> np.ndarray:
    """Quantize float array to int8 using given scale/zero_point."""
    q = np.round(x / qparams.scale + qparams.zero_point)
    q = np.clip(q, np.iinfo(qparams.dtype).min, np.iinfo(qparams.dtype).max)
    return q.astype(qparams.dtype)

def dequantize_tensor(q: np.ndarray, qparams: QuantParams) -> np.ndarray:
    """Dequantize int8 array back to float."""
    return (q.astype(np.float32) - qparams.zero_point) * qparams.scale

def choose_qparams(x: np.ndarray, num_bits=8, symmetric=True) -> QuantParams:
    """Derive scale/zero_point from tensor range."""
    qmin, qmax = -128, 127 if num_bits == 8 else (0, 255)
    xmin, xmax = float(x.min()), float(x.max())
    if symmetric:
        max_val = max(abs(xmin), abs(xmax))
        scale = max_val / ((qmax - qmin) / 2) if max_val > 0 else 1.0
        zp = 0
    else:
        scale = (xmax - xmin) / (qmax - qmin) if xmax > xmin else 1.0
        zp = int(round(qmin - xmin / scale))
    return QuantParams(scale, zp, np.int8)

def export_qparams(qparams: QuantParams, path: str):
    with open(path, "w") as f:
        json.dump({"scale": qparams.scale, "zero_point": qparams.zero_point}, f, indent=2)