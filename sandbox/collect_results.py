import subprocess
import re
import os

files = [
    ("sandbox/graph_compressor/poly_adaptive.ino", "Poly3-Adaptive"),
    ("sandbox/graph_compressor/poly_differential.ino", "Poly3-Differential"),
    ("sandbox/graph_compressor/poly_5th_adaptive.ino", "Poly5-Adaptive"),
    ("sandbox/graph_compressor/poly_5th_differential_adaptive.ino", "Poly5-Diff-Adaptive"),
    ("sandbox/graph_compressor/log_delta.ino", "Log-Delta-8Bit"),
    ("sandbox/graph_compressor/log_delta_4bit.ino", "Log-Delta-4Bit")
]

signals = ["composite_sine", "exp_decay", "noisy_sine"]

print("| Algorithm | Signal | MSE | Ratio |")
print("|-----------|--------|-----|-------|")

for f_path, label in files:
    for sig in signals:
        res = subprocess.run(["python3", "sandbox/run_test.py", f_path, sig], capture_output=True, text=True)
        mse_match = re.search(r"MSE: ([\d.e+-]+)", res.stdout)
        ratio_match = re.search(r"Ratio: ([\d.]+)", res.stdout)

        mse = mse_match.group(1) if mse_match else "N/A"
        ratio = ratio_match.group(1) if ratio_match else "N/A"

        if mse != "N/A":
            try:
                mse_f = float(mse)
                mse = f"{mse_f:.6f}"
            except:
                pass

        print(f"| {label} | {sig} | {mse} | {ratio}x |")
