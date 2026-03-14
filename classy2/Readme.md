# SensorFilter (classy2)

Advanced sensor filtering library for Arduino, featuring a 1D Kalman Filter combined with robust outlier rejection and numerically stable statistics.

## Features

- **1D Kalman Filter**: Smooths noisy sensor readings using an internal state estimate and covariance.
- **Outlier Rejection**: Automatically ignores readings that deviate significantly from the rolling window average (z-score based).
- **Numerically Stable Statistics**: Uses **Welford's Algorithm** for single-pass, numerically stable calculation of mean and variance. This is more efficient and stable than traditional sum-of-squares methods.
- **Modern C++ Architecture**: Utilizes `std::vector` for safe memory management, providing "Rule of Zero/Three" compliance and improved reliability over raw pointers.
- **Performance Optimized**: Statistics are calculated in O(N) time with minimal overhead. Benchmarked at ~3μs for window size 16 and ~15μs for window size 1000 on modern systems.
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

The filter has been evaluated against several challenging noise models:

1.  **Switching Noise**: Effectively rejects periodic or random large-amplitude spikes. See `test_switching.png`.
2.  **Ringing Response**: Handles oscillatory transients following step changes without losing track of the true signal. See `test_ringing.png`.
3.  **Shot Noise (Poisson)**: Robustly filters out intermittent high-energy spikes. See `test_shot.png`.
4.  **Heavy White Noise**: Maintains signal integrity even with a Signal-to-Noise Ratio (SNR) of ~1. See `test_white_noise.png`.
5.  **Standard Cases**: Traditional step, sine, and constant signal tests. See `test_step.png`, `test_sine.png`, `test_constant.png`.

### Performance Summary
| Window Size | Avg Time/Call (μs) |
|-------------|--------------------|
| 10          | ~2.3               |
| 100         | ~3.6               |
| 1000        | ~15.4              |
