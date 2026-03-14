import numpy as np
import subprocess
import matplotlib.pyplot as plt
import os

def run_filter(input_signals):
    # Try different paths to find the executable
    executable = './classy2/tests/filter_test'
    if not os.path.exists(executable):
        executable = './tests/filter_test'

    process = subprocess.Popen([executable], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    input_str = "\n".join(map(str, input_signals))
    stdout, stderr = process.communicate(input=input_str)
    if stderr:
        print("Error:", stderr)
    return np.array([float(x) for x in stdout.splitlines()])

def test_constant_with_noise():
    n = 200
    t = np.arange(n)
    signal = np.ones(n) * 10
    noise = np.random.normal(0, 1, n)
    input_data = signal + noise

    # Add some outliers
    input_data[50] = 50
    input_data[150] = -30

    filtered = run_filter(input_data)

    plt.figure(figsize=(12, 6))
    plt.plot(t, input_data, label='Raw Input (with noise & outliers)', alpha=0.5)
    plt.plot(t, signal, label='True Signal', linestyle='--')
    plt.plot(t, filtered, label='Filtered Output')
    plt.title('Constant Signal with Noise and Outliers')
    plt.legend()
    plt.savefig('tests/test_constant.png' if os.path.exists('tests') else 'classy2/tests/test_constant.png')
    plt.close()

def test_step_response():
    n = 200
    t = np.arange(n)
    signal = np.zeros(n)
    signal[100:] = 10
    noise = np.random.normal(0, 0.2, n)
    input_data = signal + noise

    filtered = run_filter(input_data)

    plt.figure(figsize=(12, 6))
    plt.plot(t, input_data, label='Raw Input', alpha=0.5)
    plt.plot(t, signal, label='True Signal', linestyle='--')
    plt.plot(t, filtered, label='Filtered Output')
    plt.title('Step Response')
    plt.legend()
    plt.savefig('tests/test_step.png' if os.path.exists('tests') else 'classy2/tests/test_step.png')
    plt.close()

def test_sine_wave():
    n = 400
    t = np.linspace(0, 4*np.pi, n)
    signal = 10 * np.sin(t)
    noise = np.random.normal(0, 0.5, n)
    input_data = signal + noise

    filtered = run_filter(input_data)

    plt.figure(figsize=(12, 6))
    plt.plot(t, input_data, label='Raw Input', alpha=0.5)
    plt.plot(t, signal, label='True Signal', linestyle='--')
    plt.plot(t, filtered, label='Filtered Output')
    plt.title('Sine Wave Tracking')
    plt.legend()
    plt.savefig('tests/test_sine.png' if os.path.exists('tests') else 'classy2/tests/test_sine.png')
    plt.close()

if __name__ == "__main__":
    test_constant_with_noise()
    test_step_response()
    test_sine_wave()
    print("Tests completed.")
