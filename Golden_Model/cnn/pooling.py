# cnn/pooling.py
import numpy as np
from typing import List, Tuple
from cnn.quant import quantize_tensor, QuantParams

def max_pooling(feature_map: np.ndarray,
                pool_size: int = 2,
                stride: int = 2,
                qparams: QuantParams=None) -> np.ndarray:
    """
    2D max pooling. Accepts 2D arrays (single channel).
    - pool_size: pooling window (square)
    - stride: step between windows
    Returns pooled 2D feature map.
    """
    H, W = feature_map.shape
    out_h = (H - pool_size) // stride + 1
    out_w = (W - pool_size) // stride + 1
    out = np.zeros((out_h, out_w), dtype=np.float32)

    for oy in range(out_h):
        for ox in range(out_w):
            y0 = oy * stride
            x0 = ox * stride
            window = feature_map[y0:y0 + pool_size, x0:x0 + pool_size]
            out[oy, ox] = np.max(window)

    if qparams:
        return quantize_tensor(out, qparams)
    return out

# NEW: run multiple pooling layers sequentially
def run_pooling_layers(input_map: np.ndarray,
                       pooling_layers: List[dict],
                       qparams: QuantParams=None) -> Tuple[np.ndarray, int]:
    """
    Run a sequence of pooling layers.
    pooling_layers: list of dicts, each with {"pool_size": int, "stride": int}
    Returns (final pooled map, memory_usage_bytes)
    """
    fmap = input_map
    memory_usage = 0
    for layer in pooling_layers:
        fmap = max_pooling(fmap,
                           pool_size=layer.get("pool_size", 2),
                           stride=layer.get("stride", 2),
                           qparams=qparams)
        # track memory usage (activations only, pooling has no weights)
        memory_usage += fmap.size * 4  # float32 bytes
    return fmap, memory_usage