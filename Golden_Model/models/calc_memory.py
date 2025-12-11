import os
import argparse

def count_hex_file(file_path):
    """Count number of 32-bit parameters in a hex file."""
    with open(file_path, 'r') as f:
        lines = [l.strip() for l in f.readlines() if l.strip()]

    # Each line is one 32-bit value (4 bytes)
    num_params = len(lines)
    num_bytes = num_params * 4
    return num_params, num_bytes


def analyze_directory(directory):
    total_params = 0
    total_bytes = 0
    breakdown = []

    for fname in os.listdir(directory):
        if fname.endswith(".hex"):
            path = os.path.join(directory, fname)
            params, bytes_used = count_hex_file(path)
            total_params += params
            total_bytes += bytes_used
            breakdown.append((fname, params, bytes_used))

    return total_params, total_bytes, breakdown


def print_report(total_params, total_bytes, breakdown):
    print("==== Memory Usage Report ====")
    print(f"Total parameters: {total_params:,}")
    print(f"Total memory:     {total_bytes:,} bytes")
    print(f"                 {total_bytes / 1024:.2f} KB")
    print(f"                 {total_bytes / (1024*1024):.4f} MB")
    print("\n---- Per-file Breakdown ----")
    for fname, params, bytes_used in breakdown:
        print(f"{fname:30}  {params:10,} params   {bytes_used:10,} bytes")
    print("\n=============================")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dir", required=True, help="Folder containing .hex files")
    args = parser.parse_args()

    total_params, total_bytes, breakdown = analyze_directory(args.dir)
    print_report(total_params, total_bytes, breakdown)
