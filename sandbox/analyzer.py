import sys
import matplotlib.pyplot as plt
import numpy as np
import os

def analyze(input_file, output_file, plot_name, compressed_size_bytes):
    it = []
    iv = []
    with open(input_file, 'r') as f:
        for line in f:
            parts = line.strip().split(",")
            if len(parts) == 2:
                it.append(float(parts[0]))
                iv.append(float(parts[1]))

    output_t = []
    output_v = []
    with open(output_file, 'r') as f:
        for line in f:
            if line.startswith("RESULT:"):
                parts = line.strip().split(":")
                if len(parts) == 3:
                    output_t.append(float(parts[1]))
                    output_v.append(float(parts[2]))

    it = np.array(it)
    iv = np.array(iv)
    ot = np.array(output_t)
    ov = np.array(output_v)

    plt.figure(figsize=(12, 7))
    plt.plot(it, iv, 'b-', alpha=0.6, label='Original')
    if len(ot) > 0:
        plt.plot(ot, ov, 'r--', label='Decompressed')

    # Raw size assuming 2 floats per point (8 bytes)
    raw_size = len(it) * 8
    ratio = raw_size / compressed_size_bytes if compressed_size_bytes > 0 else 0

    mse = 0
    if len(ot) > 0:
        interp_ov = np.interp(it, ot, ov)
        mse = np.mean((iv - interp_ov)**2)

    plt.legend()
    plt.title(f"Compression Analysis: {plot_name}\nMSE: {mse:.6f}, Ratio: {ratio:.2f}x")
    plt.xlabel("Time")
    plt.ylabel("Value")
    plt.grid(True)
    plt.savefig(f"{plot_name}.png")

    print(f"MSE: {mse}")
    print(f"Ratio: {ratio}")
    print(f"CompressedSize: {compressed_size_bytes}")

if __name__ == "__main__":
    # Expecting: input_csv, output_log, plot_name, compressed_size
    compressed_size = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    analyze(sys.argv[1], sys.argv[2], sys.argv[3], compressed_size)
