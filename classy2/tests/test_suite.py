import numpy as np
import subprocess
import matplotlib.pyplot as plt
import os

def run_filter(input_signals, window_size=16):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    executable = os.path.join(script_dir, './filter_test')
    process = subprocess.Popen([executable, str(window_size)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    input_str = "\n".join(map(str, input_signals))
    stdout, stderr = process.communicate(input=input_str)
    if stderr:
        print("Error:", stderr)
    return np.array([float(x) for x in stdout.splitlines()])

def add_switching_noise(signal, probability=0.05, amplitude=5.0):
    noise = np.zeros_like(signal)
    mask = np.random.random(len(signal)) < probability
    noise[mask] = np.random.choice([-amplitude, amplitude], size=np.sum(mask))
    return signal + noise

def add_ringing(signal, frequency=0.1, decay=0.05, amplitude=2.0):
    # Ringing usually happens after a step. Let's find steps.
    diff = np.diff(signal, prepend=signal[0])
    ringing = np.zeros_like(signal)
    t = np.arange(len(signal))
    for i in np.where(np.abs(diff) > 0.5)[0]:
        duration = min(len(signal) - i, 50)
        local_t = np.arange(duration)
        ringing[i:i+duration] += diff[i] * np.exp(-decay * local_t) * np.cos(frequency * local_t)
    return signal + ringing

def add_shot_noise(signal, lam=0.5, amplitude=1.0):
    noise = np.random.poisson(lam, len(signal)) * amplitude
    return signal + noise

def calculate_metrics(true_signal, filtered_signal):
    rmse = np.sqrt(np.mean((true_signal - filtered_signal)**2))
    # Correlation for lag estimation (simple)
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
    plt.savefig(filename)
    plt.close()

def test_switching_noise():
    n = 300
    t = np.arange(n)
    signal = np.ones(n) * 10
    input_data = add_switching_noise(signal, probability=0.1, amplitude=10.0)
    input_data += np.random.normal(0, 0.5, n) # add some white noise too
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Switching Noise Rejection', 'test_switching.png')

def test_ringing_response():
    n = 400
    t = np.arange(n)
    signal = np.zeros(n)
    signal[100:200] = 10
    signal[300:] = 5
    input_data = add_ringing(signal, frequency=0.5, decay=0.1)
    input_data += np.random.normal(0, 0.1, n)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Step Response with Ringing', 'test_ringing.png')

def test_shot_noise():
    n = 300
    t = np.arange(n)
    signal = np.linspace(0, 10, n)
    input_data = add_shot_noise(signal, lam=0.2, amplitude=5.0)
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'Ramp with Shot Noise', 'test_shot.png')

def test_heavy_white_noise():
    n = 500
    t = np.arange(n)
    signal = 5 * np.sin(t * 0.05)
    input_data = signal + np.random.normal(0, 5.0, n) # SNR is 1
    filtered = run_filter(input_data)
    plot_and_save(t, input_data, signal, filtered, 'High Amplitude White Noise', 'test_white_noise.png')

if __name__ == "__main__":
    test_switching_noise()
    test_ringing_response()
    test_shot_noise()
    test_heavy_white_noise()
    print("Extended tests completed.")
