# SensorFilter (classy2)

Advanced sensor filtering library featuring a **2D Kinematic Kalman Filter** (Position/Velocity) with adaptive noise scaling and robust outlier rejection.

## Features

- **2D Kinematic Model**: Tracks both signal value (Position) and its rate of change (Velocity). This enables zero-lag tracking of ramping signals and superior prediction during data dropouts compared to 1D filters.
- **Adaptive Sage-Husa Algorithm**: Dynamically estimates measurement noise ($R$) from innovation statistics. It automatically adjusts smoothing strength based on the real-time noise environment.
- **Maneuver Detection (Adaptive Q)**: Scales process noise ($Q$) based on innovation bias. The filter becomes "agile" when it detects a consistent trend (like a step or fast ramp) and "smooth" when the signal is stable.
- **Robust Outlier Rejection**: Employs a prediction-based rejection window. It ignores transient spikes (shot noise) that deviate too far from the kinematic prediction.
- **Momentum-Aware Step Detection**: If a signal shift persists for multiple samples (default: 5), the filter "snaps" to the new value. It automatically resets internal velocity to prevent "momentum poisoning" and overshoot following large noise bursts.
- **State Persistence**: Supports `getState()` and `setState()` for persisting filter covariance and estimates in low-power modes (e.g., ESP32 RTC memory).
- **Numerical Stability**: Implements **Kahan Summation** for windowed variance calculations and strict covariance floors to prevent underflow on microcontrollers.
- **Performance Optimized**: Uses pure `float` math and squared-space comparisons to eliminate expensive `sqrt()` calls in the main update loop.
- **Verification Suite**: Integrated Python-based signal generator and analyzer for evaluating performance against complex noise models.

## Files

- `SensorFilter.h/cpp`: The core library.
- `example.ino`: Arduino example sketch.
- `Makefile`: For building the test suite on a PC.
- `tests/`: Contains the test runner, mock environment, and Python test suite.

## How to Use (Arduino)

```cpp
#include "SensorFilter.h"

// Params: window size (16), outlier threshold (2.5 sigma),
//         q_pos (0.01), q_vel (0.001), r_noise (0.1)
SensorFilter filter(16, 2.5f, 0.01f, 0.001f, 0.1f);

void loop() {
    float rawValue = (float)analogRead(A0) * (5.0f / 1023.0f);

    // Update filter (returns smoothed value)
    float filteredValue = filter.updateSensorReading(rawValue);

    // Optional: Get estimated rate of change
    float velocity = filter.getVelocity();

    Serial.print(">Raw:"); Serial.println(rawValue);
    Serial.print(">Filtered:"); Serial.println(filteredValue);
    delay(20);
}
```

## Testing and Benchmarking

The project includes a comprehensive test suite using Python to generate synthetic signals and analyze the filter's performance.

### Running Functional Tests
1.  Navigate to the directory: `cd classy2`
2.  Run the tests: `make && python3 tests/test_suite.py`
3.  View the results in the `test_results/` directory (PNG graphs).

### Running Benchmarks
Use the provided benchmarking tool to measure execution time across different window sizes:
`python3 tests/benchmark_filter.py`

## Test Results

The v3.0 filter has been evaluated against several challenging scenarios. Detailed graphs (with RMSE and Lag metrics) are in `test_results/`:

1.  **High Dynamic Range (HDR) Steps**: Tested with transitions from 0 to 1000. Shows rapid acquisition within 5 samples. (`test_hdr.png`)
2.  **Ramp with Shot Noise**: Evaluates 2D tracking. Zero-lag ramp following while ignoring 15.0+ magnitude spikes. (`test_shot.png`)
3.  **Signal Dropouts**: Uses the kinematic model to "bridge" missing data points using the last known velocity. (`test_dropout.png`)
4.  **Heavy White Noise**: Tested at SNR=1. Maintains a stable estimate of the underlying sine wave with minimal phase lag. (`test_white_noise.png`)
5.  **Chirp/Frequency Sweep**: Tracks signals with increasing frequency, demonstrating the limits of the adaptive Q-scaling. (`test_chirp.png`)
6.  **Switching Noise**: Effectively rejects periodic or random large-amplitude spikes. (`test_switching.png`)
7.  **Ringing Response**: Handles oscillatory transients following step changes without losing track of the true signal. (`test_ringing.png`)

### Performance Metrics (Desktop x64)
- **Main Update Loop**: ~0.8μs per call.
- **With Windowed R-Estimation (Size 50)**: ~2.1μs per call.
