import numpy as np
from cnn.conv import conv2d_naive, conv2d_numpy, conv2d_multi_channel, run_conv_layers
from cnn.quant import choose_qparams, quantize_tensor, dequantize_tensor

def test_quantization_roundtrip():
    """Check that quantize->dequantize roundtrip stays close to float output."""
    img = np.random.randn(8, 8).astype(np.float32)
    kernel = np.ones((3, 3), dtype=np.float32)
    conv_out = conv2d_naive(img, kernel, stride=1, padding=1)

    qparams = choose_qparams(conv_out)
    conv_q = quantize_tensor(conv_out, qparams)
    deq = dequantize_tensor(conv_q, qparams)

    # Allow tolerance equal to one quantization step
    assert np.allclose(conv_out, deq, atol=qparams.scale), \
        f"Quantization roundtrip failed. Max diff: {np.abs(conv_out - deq).max()}"

def test_single_channel():
    """Compare naive conv vs numpy conv on a deterministic single-channel input."""
    img = np.array([
        [1, 2, 3, 0, 1],
        [4, 6, 6, 2, 1],
        [3, 2, 1, 0, 2],
        [1, 2, 2, 2, 1],
        [0, 1, 1, 2, 3]
    ], dtype=np.float32)

    kernel = np.array([
        [1, 0, -1],
        [1, 0, -1],
        [1, 0, -1]
    ], dtype=np.float32)

    out_naive = conv2d_naive(img, kernel, stride=1, padding=0)
    out_ref = conv2d_numpy(img, kernel, stride=1, padding=0)

    assert np.allclose(out_naive, out_ref, atol=1e-6), \
        f"Single-channel conv mismatch.\nnaive:\n{out_naive}\nref:\n{out_ref}"

def test_multi_channel():
    """Check multi-channel conv matches sum of per-channel convs."""
    base = np.arange(25).reshape(5, 5).astype(np.float32)
    img = np.stack([base, base * 2, base * 3], axis=-1)  # H,W,3

    k = np.zeros((3, 3, 3), dtype=np.float32)
    k[:, :, 0] = np.array([[1, 0, -1],
                           [1, 0, -1],
                           [1, 0, -1]])
    k[:, :, 1] = k[:, :, 0] * 0.5
    k[:, :, 2] = k[:, :, 0] * -0.25

    out = conv2d_multi_channel(img, k, bias=1.0, stride=1, padding=0)

    # Reference: sum of per-channel convs + bias
    accum = None
    for ch in range(3):
        ch_out = conv2d_numpy(img[:, :, ch], k[:, :, ch], stride=1, padding=0)
        accum = ch_out if accum is None else accum + ch_out
    accum = accum + 1.0  # bias

    # conv2d_multi_channel now returns (H,W,1), so compare against accum[...,None]
    assert np.allclose(out, accum[..., None], atol=1e-6), \
        f"Multi-channel conv mismatch.\nmulti:\n{out}\naccum:\n{accum}"

def test_run_conv_layers():
    """Check multi-layer runner executes sequential convs and reports memory usage."""
    img = np.random.randn(8, 8, 1).astype(np.float32)  # single-channel input

    # Define two conv layers with different kernels/biases
    layer1 = {
        "kernel": np.ones((3, 3, 1), dtype=np.float32),
        "bias": 0.5,
        "stride": 1,
        "padding": 1
    }
    layer2 = {
        "kernel": -1.0 * np.ones((3, 3, 1), dtype=np.float32),
        "bias": -0.5,
        "stride": 1,
        "padding": 1
    }

    fmap, mem_usage = run_conv_layers(img, [layer1, layer2], apply_relu=True)

    print("run_conv_layers output shape:", fmap.shape)
    print("Estimated memory usage (bytes):", mem_usage)
    # fmap should always be H,W,C
    assert fmap.ndim == 3, "Unexpected output shape from run_conv_layers"

if __name__ == "__main__":
    test_quantization_roundtrip()
    test_single_channel()
    test_multi_channel()
    test_run_conv_layers()
    print("All conv tests passed.")