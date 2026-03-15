import subprocess
import sys
import os
import re
import shutil

def run_test(ino_path, signal_type):
    print(f"--- Testing {ino_path} with {signal_type} ---")

    # 1. Generate signal
    subprocess.run(["python3", "sandbox/signal_gen.py", signal_type, "input.csv"])

    # 2. Build and run
    try:
        with open(ino_path, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
    except Exception as e:
        print(f"Error reading file: {e}")
        return False

    # Check for truncation (simple heuristic: no closing brace at the end of file)
    if content.strip() and not content.strip().endswith('}') and not content.strip().endswith(';') and not content.strip().endswith('>'):
         # This is very rough, but better than nothing
         pass

    # Ensure it includes Arduino.h if missing
    if "#include <Arduino.h>" not in content:
        content = "#include <Arduino.h>\n" + content

    with open("sandbox/mock_arduino/ino_content.cpp", "w") as f:
        f.write(content)

    # Copy associated .cpp/.hpp files from the same directory
    src_dir = os.path.dirname(ino_path)
    extra_srcs = []
    if src_dir:
        for f in os.listdir(src_dir):
            if f.endswith(".cpp") or f.endswith(".hpp") or f.endswith(".h"):
                if f != os.path.basename(ino_path):
                    shutil.copy(os.path.join(src_dir, f), "sandbox/mock_arduino/")
                    if f.endswith(".cpp"):
                        extra_srcs.append(os.path.join("sandbox/mock_arduino", f))

    # Compile
    build_dir = "test_build"
    os.makedirs(build_dir, exist_ok=True)
    compile_cmd = [
        "g++", "-I", "sandbox/mock_arduino",
        "-I", "/usr/include/eigen3",
        "sandbox/mock_arduino/Arduino.cpp",
        "sandbox/mock_arduino/main.cpp",
        "sandbox/mock_arduino/ino_content.cpp"
    ] + extra_srcs + [
        "-o", f"{build_dir}/test_app"
    ]

    res = subprocess.run(compile_cmd, capture_output=True, text=True)
    if res.returncode != 0:
        print("Compilation failed!")
        # Save stderr for debugging
        with open("compile_error.log", "w") as f:
            f.write(res.stderr)
        return False

    # Run
    try:
        with open("input.csv", "r") as fin:
            res = subprocess.run([f"./{build_dir}/test_app"], stdin=fin, capture_output=True, text=True, timeout=5)
    except subprocess.TimeoutExpired:
        print("Execution timed out!")
        return False
    except Exception as e:
        print(f"Execution error: {e}")
        return False

    if res.returncode != 0:
        print(f"Execution failed with return code {res.returncode}")
        with open("exec_error.log", "w") as f:
            f.write(res.stderr)
        return False

    output_log = res.stdout
    with open("test_output.log", "w") as fout:
        fout.write(output_log)

    # Extract compressed size
    size_match = re.search(r"SIZE:(\d+)", output_log)
    compressed_size = int(size_match.group(1)) if size_match else 0

    # 3. Analyze
    name = os.path.basename(ino_path).replace(".ino", "").replace(".cpp", "")
    plot_path = f"eval_{name}_{signal_type}"
    subprocess.run(["python3", "sandbox/analyzer.py", "input.csv", "test_output.log", plot_path, str(compressed_size)])
    return True

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 run_test.py <ino_path> <signal_type>")
    else:
        run_test(sys.argv[1], sys.argv[2])
