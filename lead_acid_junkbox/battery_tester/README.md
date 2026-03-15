# Lead-Acid Battery Tester Test Suite

This directory contains a host-based test suite for the `BatteryTester` C++ logic. It uses a Python-based physical emulator to simulate lead-acid battery dynamics and verify the controller's behavior.

## Directory Structure

- `src/`: Contains the core `BatteryTester` C++ class.
- `tests/`: Contains the testing infrastructure.
  - `emulator.py`: 3-cell lead-acid battery model with SoC tracking, internal resistance, and gassing plateaus.
  - `host_main.cpp`: A wrapper that allows the C++ logic to communicate via standard I/O.
  - `run_tests.py`: Orchestrates the simulation by bridging the C++ process and the Python model.
  - `plot_results.py`: Generates `test_results.png` from simulation data.
  - `analyze_csv.py`: Helper script to verify state transitions and thresholds.
  - `Makefile`: Compiles the host tester.

## How to Run

1. Navigate to the `tests/` directory.
2. Run `make` to compile the host tester.
3. Run `python3 run_tests.py` to execute the simulation.
4. Run `python3 plot_results.py` to visualize the performance.

## Controller Features Tested

- **Kalman Filter OCV Estimation:** Extracts the Open Circuit Voltage from noisy measurements by compensating for IR drop.
- **Slope Detection:** Detects the end-of-charge "gassing plateau" where voltage rise slows down.
- **PID Current Control:** Maintains a target charging current using a PWM-controlled source.
- **Multi-Cycle Charging:** Gradually reduces charging current in steps (e.g., 50mA) after each gassing detection to safely top off the battery.
- **Anti-Windup & Debouncing:** Includes logic to prevent PID integral windup and false triggers due to noise.
