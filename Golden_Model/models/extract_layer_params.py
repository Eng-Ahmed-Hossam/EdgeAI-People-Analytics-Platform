import argparse
import math

def parse_cfg(cfg_path):
    layers = []
    current = {}

    with open(cfg_path, 'r') as f:
        for line in f:
            line = line.strip()
            if len(line) == 0 or line.startswith('#'):
                continue

            if line.startswith('['):
                if current:
                    layers.append(current)
                current = {'type': line.strip('[]')}
            else:
                key, value = line.split('=')
                current[key.strip()] = value.strip()

    if current:
        layers.append(current)

    return layers


def compute_output_size(inp, k, stride, pad):
    """Compute output size for conv/pool."""
    return math.floor((inp + 2 * pad - k) / stride + 1)


def summarize(cfg_path, input_resolution):
    layers = parse_cfg(cfg_path)

    in_h = input_resolution
    in_w = input_resolution
    in_channels = 3

    print("===== CNN Layer Geometry =====")
    print(f"Input: {in_h} x {in_w} x {in_channels}")
    print("")

    for idx, layer in enumerate(layers):
        print(f"--- Layer {idx} ({layer['type']}) ---")

        if layer["type"] == "convolutional":
            k = int(layer['size'])
            stride = int(layer['stride'])
            pad = int(layer.get("pad", 0))
            filters = int(layer['filters'])

            # Darknet uses pad=1 → auto padding=k//2
            if pad == 1:
                pad = k // 2

            out_h = compute_output_size(in_h, k, stride, pad)
            out_w = compute_output_size(in_w, k, stride, pad)

            print(f"Input shape:  {in_h} x {in_w} x {in_channels}")
            print(f"Kernel size:  {k}x{k}")
            print(f"Stride:       {stride}")
            print(f"Padding:      {pad}")
            print(f"Filters:      {filters}")
            print(f"Output shape: {out_h} x {out_w} x {filters}")

            # Update next layer input
            in_h, in_w, in_channels = out_h, out_w, filters

        elif layer['type'] == 'maxpool':
            stride = int(layer['stride'])
            size = int(layer['size'])

            out_h = compute_output_size(in_h, size, stride, 0)
            out_w = compute_output_size(in_w, size, stride, 0)

            print(f"MaxPool size: {size} stride {stride}")
            print(f"Input:        {in_h} x {in_w} x {in_channels}")
            print(f"Output:       {out_h} x {out_w} x {in_channels}")

            in_h, in_w = out_h, out_w

        elif layer['type'] == 'upsample':
            stride = int(layer['stride'])

            out_h = in_h * stride
            out_w = in_w * stride

            print(f"Upsample by {stride}")
            print(f"Input:   {in_h} x {in_w} x {in_channels}")
            print(f"Output:  {out_h} x {out_w} x {in_channels}")

            in_h, in_w = out_h, out_w

        else:
            print("(Skipped layer type)")

        print("")

    print("===== END =====")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", required=True, help="Path to Tiny YOLO .cfg")
    parser.add_argument("--input", type=int, default=416,
                        help="Input resolution (assumes square input)")

    args = parser.parse_args()
    summarize(args.cfg, args.input)
