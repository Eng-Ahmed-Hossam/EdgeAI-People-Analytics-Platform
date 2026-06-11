# cnn/relu.py
import numpy as np
from cnn.quant import quantize_tensor, QuantParams

def relu(feature_map: np.ndarray, qparams: QuantParams=None) -> np.ndarray:
    """
    Single-step ReLU activation.
    Mirrors hardware: result = max(0, x)
    Accepts numpy arrays of any shape.
    """
    out = np.maximum(feature_map, 0)

    if qparams:
        return quantize_tensor(out, qparams)
    return out

