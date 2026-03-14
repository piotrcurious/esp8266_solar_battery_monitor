# SensorFilter (classy2)

Advanced sensor filtering library for Arduino, featuring an **Adaptive 1D Kalman Filter** combined with robust outlier rejection and numerically stable statistics.

## Features

- **Adaptive Kalman Filter**: Smooths noisy sensor readings using an internal state estimate. Includes a simplified Sage-Husa adaptation that dynamically adjusts measurement noise ($R$) based on innovation variance, allowing it to respond better to changing noise environments.
- **"Clean Window" Outlier Rejection**: Automatically ignores readings that deviate significantly from the rolling window average (z-score based). Unlike standard filters, when an outlier is detected, the rolling buffer is updated with the *current filtered estimate* instead of the outlier. This prevents "baseline poisoning" where a series of spikes would otherwise inflate the variance and compromise future rejection.
- **Step Detection**: Integrated logic to detect genuine signal shifts. If a threshold of consecutive outliers is met, the filter "snaps" to the new value and resets the statistical buffer, ensuring low latency during state transitions.
- **Numerically Stable Statistics**: Uses **Welford's Algorithm** for single-pass, numerically stable calculation of mean and variance.
- **Modern C++ Architecture**: Utilizes `std::vector` for safe memory management, providing "Rule of Zero/Three" compliance and improved reliability over raw pointers.
- **Performance Optimized**: Optimized for MCU execution. Uses pure `float` math to avoid expensive `double` conversions, and eliminates `sqrt()` in the hot path by performing comparisons in squared-space. Statistics are calculated in O(N) time.
- **Extensible Testing**: Integrated Python-based test suite for signal generation and analysis.

## Files

- `SensorFilter.h/cpp`: The core library.
- `example.ino`: Arduino example sketch.
- `Makefile`: For building the test suite on a PC.
- `tests/`: Contains the test runner, mock environment, and Python test suite.

## How to Use (Arduino)

```cpp
#include "SensorFilter.h"

// Initialize with: window size, outlier threshold (std dev), process noise, measurement noise
SensorFilter filter(16, 2.0, 0.1, 1.0);

void loop() {
    float rawValue = analogRead(A0);
    float filteredValue = filter.updateSensorReading(rawValue);
    Serial.println(filteredValue);
    delay(10);
}
```

## Testing and Benchmarking

The project includes a comprehensive test suite using Python to generate synthetic signals and analyze the filter's performance.

### Running Functional Tests
1.  Navigate to the directory: `cd classy2`
2.  Run the tests: `make && python3 tests/test_suite.py`
3.  View the results in the `tests/` directory (PNG graphs).

### Running Benchmarks
Use the provided benchmarking tool to measure execution time across different window sizes:
`python3 tests/benchmark_filter.py`

## Test Results

The filter has been evaluated against several challenging noise models. Evaluation graphs (including RMSE and Lag metrics) can be found in the `test_results/` directory:

1.  **Switching Noise**: Effectively rejects periodic or random large-amplitude spikes. See `test_results/test_switching.png`.
2.  **Ringing Response**: Handles oscillatory transients following step changes without losing track of the true signal. See `test_results/test_ringing.png`.
3.  **Shot Noise (Poisson)**: Robustly filters out intermittent high-energy spikes. See `test_results/test_shot.png`.
4.  **Heavy White Noise**: Maintains signal integrity even with a Signal-to-Noise Ratio (SNR) of ~1. See `test_results/test_white_noise.png`.

### Performance Summary
| Window Size | Avg Time/Call (μs) |
|-------------|--------------------|
| 10          | ~2.3               |
| 100         | ~3.6               |
| 1000        | ~15.4              |
