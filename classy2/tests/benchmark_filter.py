import time
import subprocess
import os

def benchmark():
    # Detect if we are in classy2/tests or classy2 or root
    current_dir = os.getcwd()
    if os.path.basename(current_dir) == 'tests':
        os.chdir('..')

    # Now we should be in classy2
    if not os.path.exists('Makefile'):
        # Try to find classy2
        if os.path.exists('classy2'):
            os.chdir('classy2')

    # Ensure test_runner is built
    os.system("make")

    window_sizes = [10, 50, 100, 500, 1000]
    iterations = 50000

    print(f"{'Window Size':<15} | {'Total Time (s)':<15} | {'Avg Time/Call (us)':<20}")
    print("-" * 55)

    input_data = "\n".join(["1.0"] * iterations)

    executable = "./tests/filter_test"

    for size in window_sizes:
        # Pass size as argument to filter_test
        start_time = time.time()
        process = subprocess.Popen([executable, str(size)],
                                   stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        process.communicate(input=input_data)
        end_time = time.time()

        total_time = end_time - start_time
        avg_time = (total_time / iterations) * 1e6

        print(f"{size:<15} | {total_time:<15.4f} | {avg_time:<20.2f}")

if __name__ == "__main__":
    benchmark()
