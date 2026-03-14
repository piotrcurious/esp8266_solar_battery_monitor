# SensorFilter (classy2)

Advanced sensor filtering library for Arduino, featuring a 1D Kalman Filter combined with outlier rejection and numerically stable statistics.

## Features

- **1D Kalman Filter**: Smooths noisy sensor readings using an internal state estimate and covariance.
- **Outlier Rejection**: Automatically ignores readings that deviate significantly from the rolling window average (standard deviation based).
- **Numerically Stable Statistics**: Uses **Kahan summation** to calculate the mean and standard deviation, minimizing floating-point precision errors over time.
- **Rule of Three**: Safely handles dynamic memory with copy constructor and assignment operator.
- **Input Validation**: Ensures robust behavior even with invalid configuration parameters.

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

## Running Tests

To run the full test suite and generate evaluation graphs, you need Python with `numpy` and `matplotlib`.

1.  Navigate to the directory:
    ```bash
    cd classy2
    ```
2.  Run the tests:
    ```bash
    make
    python3 tests/test_suite.py
    ```
3.  Check the `tests/` directory for generated PNG graphs.

## Test Results

The filter has been tested against three scenarios:
1.  **Constant Signal with Noise and Outliers**: Demonstrates excellent rejection of spikes (outliers) while maintaining a steady estimate.
2.  **Step Response**: Shows the filter's convergence speed when the underlying signal changes abruptly.
3.  **Sine Wave Tracking**: Verifies the filter's ability to track dynamic signals with minimal lag.

Graphs can be found in `tests/test_constant.png`, `tests/test_step.png`, and `tests/test_sine.png`.
