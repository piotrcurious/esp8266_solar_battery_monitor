import numpy as np
import subprocess
import matplotlib.pyplot as plt
import os

def run_filter(input_signals, window_size=16, threshold=2.0, q_pos=0.01, q_vel=0.001, r=0.1):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Executable is built by Makefile in this directory
    executable = os.path.join(script_dir, 'filter_test')
    process = subprocess.Popen([executable, str(window_size), str(threshold), str(q_pos), str(q_vel), str(r)],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    input_str = "\n".join(map(str, input_signals))
    stdout, stderr = process.communicate(input=input_str)
    if stderr:
        print("Error:", stderr)
    return np.array([float(x) for x in stdout.splitlines()])

def calculate_metrics(true_signal, filtered_signal):
    rmse = np.sqrt(np.mean((true_signal - filtered_signal)**2))
    # Correlation for lag estimation
    correlation = np.correlate(filtered_signal - np.mean(filtered_signal), true_signal - np.mean(true_signal), mode='full')
    lag = correlation.argmax() - (len(true_signal) - 1)
    return rmse, lag

def plot_and_save(t, input_data, signal, filtered, title, filename):
    rmse, lag = calculate_metrics(signal, filtered)
    plt.figure(figsize=(12, 6))
    plt.plot(t, input_data, label='Raw Input', alpha=0.5)
    plt.plot(t, signal, label='True Signal', linestyle='--')
    plt.plot(t, filtered, label='Filtered Output')
    plt.title(f"{title}\nRMSE: {rmse:.4f}, Lag: {lag}")
    plt.legend()

    # Ensure test_results directory exists in the parent classy2 root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    results_dir = os.path.join(script_dir, '../test_results')
    os.makedirs(results_dir, exist_ok=True)
    plt.savefig(os.path.join(results_dir, filename))
    plt.close()

def test_switching_noise():
    n = 300
    t = np.arange(n)
    signal = np.ones(n) * 10
    input_data = signal.copy()
    mask = np.random.random(n) < 0.1
    input_data[mask] += np.random.choice([-15.0, 15.0], size=np.sum(mask))
    input_data += np.random.normal(0, 0.5, n)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Switching Noise Rejection', 'test_switching.png')

def test_ringing_response():
    n = 400
    t = np.arange(n)
    signal = np.zeros(n)
    signal[100:200] = 10
    signal[300:] = 5
    input_data = signal.copy()
    diff = np.diff(signal, prepend=signal[0])
    for i in np.where(np.abs(diff) > 0.5)[0]:
        duration = min(len(signal) - i, 50)
        local_t = np.arange(duration)
        input_data[i:i+duration] += diff[i] * np.exp(-0.1 * local_t) * np.cos(0.5 * local_t)
    input_data += np.random.normal(0, 0.1, n)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Step Response with Ringing', 'test_ringing.png')

def test_shot_noise():
    n = 300
    t = np.arange(n)
    signal = np.linspace(0, 10, n)
    input_data = signal + np.random.poisson(0.2, n) * 5.0
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Ramp with Shot Noise', 'test_shot.png')

def test_heavy_white_noise():
    n = 500
    t = np.arange(n)
    signal = 5 * np.sin(t * 0.05)
    input_data = signal + np.random.normal(0, 5.0, n)
    filtered = run_filter(input_data, r=1.0)
    plot_and_save(t, input_data, signal, filtered, 'High Amplitude White Noise', 'test_white_noise.png')

def test_signal_dropout():
    n = 400
    t = np.arange(n)
    signal = 10 * np.sin(t * 0.02)
    input_data = signal.copy()
    input_data[150:200] = 0
    input_data += np.random.normal(0, 0.2, n)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Signal Dropout Handling', 'test_dropout.png')

def test_chirp_signal():
    n = 600
    t = np.arange(n)
    signal = 10 * np.sin(0.0001 * t**2)
    input_data = signal + np.random.normal(0, 1.0, n)
    filtered = run_filter(input_data, q_pos=0.05, q_vel=0.005)
    plot_and_save(t, input_data, signal, filtered, 'Chirp Signal Tracking', 'test_chirp.png')

def test_high_dynamic_range():
    n = 400
    t = np.arange(n)
    signal = np.zeros(n)
    signal[100:300] = 1000.0
    input_data = signal + np.random.normal(0, 10.0, n)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'High Dynamic Range Step', 'test_hdr.png')

if __name__ == "__main__":
    test_switching_noise()
    test_ringing_response()
    test_shot_noise()
    test_heavy_white_noise()
    test_signal_dropout()
    test_chirp_signal()
    test_high_dynamic_range()
    print("Extended tests with new signals completed.")
