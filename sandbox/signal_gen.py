import numpy as np
import sys

def generate_signal(type, n_points, duration):
    t = np.linspace(0, duration, n_points)
    if type == 'sine':
        v = np.sin(t * 0.5)
    elif type == 'battery':
        v = 12.6 - 0.1 * t - 0.5 * np.exp(t - duration)
    elif type == 'step':
        v = np.where(t < duration/2, 12.0, 11.0)
    elif type == 'composite_sine':
        v = np.sin(t * 0.5) + 0.5 * np.sin(t * 2.0) + 0.2 * np.sin(t * 5.0)
    elif type == 'exp_decay':
        v = 10 * np.exp(-t/10)
        for event_t in [10, 25, 45]:
            v += np.where(t > event_t, 5 * np.exp(-(t-event_t)/5), 0)
    elif type == 'noisy_sine':
        noise_level = np.linspace(0.01, 0.5, n_points)
        v = np.sin(t * 0.5) + np.random.normal(0, noise_level)
    else:
        v = np.zeros_like(t)
    return t, v

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 signal_gen.py <type> <output_file> [n_points] [duration]")
        sys.exit(1)

    sig_type = sys.argv[1]
    output_file = sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 256
    dur = float(sys.argv[4]) if len(sys.argv) > 4 else 32.0

    t, v = generate_signal(sig_type, n, dur)
    with open(output_file, 'w') as f:
        for ti, vi in zip(t, v):
            f.write(f"{ti},{vi}\n")
