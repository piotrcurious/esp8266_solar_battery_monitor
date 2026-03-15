#ifndef BATTERY_TESTER_HPP
#define BATTERY_TESTER_HPP

class BatteryTester {
public:
    BatteryTester();
    void update(unsigned long currentTimeMs, float measuredVoltage, float measuredCurrent);

    // Getters for state/debug
    float getEstimatedVoltage() const { return voltage_est; }
    float getEstimatedSlope() const { return slope_est; }
    int getState() const { return state; }
    int getPwmOutput() const { return pwmOutput; }
    float getTargetCurrent() const { return setCurrent; }

    void startTest();

private:
    void kalmanUpdate(float current_input, float measured_voltage);
    bool checkOutgassing(float voltageSlope);
    void computePID(float measuredCurrent);

    // Kalman Filter Variables
    float voltage_est = 0.0;
    float slope_est = 0.0;
    float internalResistance = 0.06; // Estimated R for 3 cells
    float deltaT = 1.0; // Seconds

    // Kalman Filter Matrices (Simplified for 2x2)
    float P[2][2] = {{1, 0}, {0, 1}};
    float Q[2][2] = {{1e-8, 0}, {0, 1e-10}};
    float R_noise = 0.005;

    // State machine
    int state = 0; // 0: Idle, 1: Testing, 2: Waiting
    unsigned long stateStartTime = 0;
    bool testInProgress = false;

    // Control
    float setCurrent = 500.0; // mA
    int pwmOutput = 0;
    float slopeThreshold = 0.0002;
    int outgassingCounter = 0;

    // Simple PID components
    float kp = 0.05;
    float ki = 0.02;
    float integralError = 0.0;
    unsigned long lastUpdateTime = 0;
};

#endif
