import subprocess
import sys
import os
import re

def run_test(ino_path, signal_type):
    print(f"--- Testing {ino_path} with {signal_type} ---")

    # 1. Generate signal
    subprocess.run(["python3", "sandbox/signal_gen.py", signal_type, "input.csv"])

    # 2. Build and run
    with open(ino_path, 'r') as f:
        content = f.read()

    # Ensure it includes Arduino.h if missing
    if "#include <Arduino.h>" not in content:
        content = "#include <Arduino.h>\n" + content

    with open("sandbox/mock_arduino/ino_content.cpp", "w") as f:
        f.write(content)

    # Compile
    build_dir = "test_build"
    os.makedirs(build_dir, exist_ok=True)
    compile_cmd = [
        "g++", "-I", "sandbox/mock_arduino",
        "sandbox/mock_arduino/Arduino.cpp",
        "sandbox/mock_arduino/main.cpp",
        "sandbox/mock_arduino/ino_content.cpp",
        "-o", f"{build_dir}/test_app"
    ]
    res = subprocess.run(compile_cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print("Compilation failed!")
        print(res.stderr)
        return

    # Run
    with open("input.csv", "r") as fin:
        res = subprocess.run([f"./{build_dir}/test_app"], stdin=fin, capture_output=True, text=True)

    if res.returncode != 0:
        print("Execution failed!")
        print(res.stderr)
        return

    output_log = res.stdout
    with open("test_output.log", "w") as fout:
        fout.write(output_log)

    # Extract compressed size from log if present, else estimate
    # We expect the .ino to print "SIZE:<bytes>"
    size_match = re.search(r"SIZE:(\d+)", output_log)
    compressed_size = int(size_match.group(1)) if size_match else 0

    # 3. Analyze
    name = os.path.basename(ino_path).replace(".ino", "")
    plot_path = f"eval_{name}_{signal_type}"
    subprocess.run(["python3", "sandbox/analyzer.py", "input.csv", "test_output.log", plot_path, str(compressed_size)])
    print(f"Results saved to {plot_path}.png")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 run_test.py <ino_path> <signal_type>")
    else:
        run_test(sys.argv[1], sys.argv[2])
