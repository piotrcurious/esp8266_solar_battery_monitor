#include "BatteryTester.hpp"
#include <cmath>
#include <algorithm>

BatteryTester::BatteryTester() {
    voltage_est = 0.0;
    slope_est = 0.0;
    state = 0;
    testInProgress = false;
    setCurrent = 500.0;
    lastUpdateTime = 0;
}

void BatteryTester::startTest() {
    if (!testInProgress) {
        testInProgress = true;
        state = 1;
        stateStartTime = 0; // Will be set on first update
    }
}

void BatteryTester::update(unsigned long currentTimeMs, float measuredVoltage, float measuredCurrent) {
    if (lastUpdateTime == 0) {
        lastUpdateTime = currentTimeMs;
        voltage_est = measuredVoltage;
        if (stateStartTime == 0) stateStartTime = currentTimeMs;
        return;
    }

    float dt = (currentTimeMs - lastUpdateTime) / 1000.0;
    if (dt <= 0) return;
    deltaT = dt;
    lastUpdateTime = currentTimeMs;

    if (testInProgress) {
        // Update Kalman Filter
        kalmanUpdate(measuredCurrent / 1000.0, measuredVoltage); // mA to A

        switch (state) {
            case 1: // Testing
                computePID(measuredCurrent);

                // Add a settling period of 30 seconds before checking outgassing
                if (currentTimeMs - stateStartTime > 30000) {
                    if (checkOutgassing(slope_est)) {
                        outgassingCounter++;
                        if (outgassingCounter > 60) { // Must be steady for 60 seconds
                            state = 2;
                            stateStartTime = currentTimeMs;
                            pwmOutput = 0;
                            integralError = 0; // Reset PID integral
                            outgassingCounter = 0;
                        }
                    } else {
                        outgassingCounter = 0;
                    }
                }
                break;

            case 2: // Waiting
                pwmOutput = 0;
                if (currentTimeMs - stateStartTime >= 20 * 60 * 1000) {
                    setCurrent -= 50;
                    if (setCurrent < 0) setCurrent = 0;
                    state = 1;
                    stateStartTime = currentTimeMs;
                }
                break;
        }
    } else {
        pwmOutput = 0;
    }
}

void BatteryTester::kalmanUpdate(float current_input, float measured_voltage) {
    // Compensate for internal resistance IR drop to get OCV estimate
    float ocv_measured = measured_voltage - current_input * internalResistance;

    // State transition matrix A:
    // x_pred = A * x
    float x_pred_v = voltage_est + deltaT * slope_est;
    float x_pred_s = slope_est;

    // P_pred = A * P * A^T + Q
    float P_pred[2][2];
    P_pred[0][0] = P[0][0] + deltaT * (P[1][0] + P[0][1] + deltaT * P[1][1]) + Q[0][0];
    P_pred[0][1] = P[0][1] + deltaT * P[1][1] + Q[0][1];
    P_pred[1][0] = P[1][0] + deltaT * P[1][1] + Q[1][0];
    P_pred[1][1] = P[1][1] + Q[1][1];

    // Innovation (using IR-compensated voltage)
    float y = ocv_measured - x_pred_v;

    // S = H * P_pred * H^T + R
    // Increase R_noise if measurement is very noisy
    float S = P_pred[0][0] + R_noise;

    // K = P_pred * H^T * inv(S)
    float K[2];
    K[0] = P_pred[0][0] / S;
    K[1] = P_pred[1][0] / S;

    // Update state
    voltage_est = x_pred_v + K[0] * y;
    slope_est = x_pred_s + K[1] * y;

    // Update covariance
    float P_new[2][2];
    P_new[0][0] = (1 - K[0]) * P_pred[0][0];
    P_new[0][1] = (1 - K[0]) * P_pred[0][1];
    P_new[1][0] = -K[1] * P_pred[0][0] + P_pred[1][0];
    P_new[1][1] = -K[1] * P_pred[0][1] + P_pred[1][1];

    P[0][0] = P_new[0][0];
    P[0][1] = P_new[0][1];
    P[1][0] = P_new[1][0];
    P[1][1] = P_new[1][1];
}

bool BatteryTester::checkOutgassing(float voltageSlope) {
    // We want to detect when the voltage stops rising (plateau) while charging.
    // The slopeThreshold should be small but positive.
    // If it's too small, noise might trigger it.
    // We only check if voltage is high enough.
    // For 3 cells, gassing plateau is often around 7.2V - 7.5V.
    return (voltage_est > 7.0) && (voltageSlope < slopeThreshold) && (voltageSlope > -0.00001);
}

void BatteryTester::computePID(float measuredCurrent) {
    float error = setCurrent - measuredCurrent;

    // Use smaller gains for stability in 1Hz loop
    // Also, if PWM is already at max/min, don't accumulate integral in that direction (better anti-windup)
    bool saturated = (pwmOutput >= 255 && error > 0) || (pwmOutput <= 0 && error < 0);
    if (!saturated) {
        integralError += error * deltaT;
    }

    // Clamp integral to avoid massive windup, but large enough to cover range
    // Max PWM is 255. If ki=0.05, 5100 * 0.05 = 255.
    if (integralError > 6000) integralError = 6000;
    if (integralError < -6000) integralError = -6000;

    float output = kp * error + ki * integralError;

    pwmOutput = (int)output;
    if (pwmOutput > 255) pwmOutput = 255;
    if (pwmOutput < 0) pwmOutput = 0;
}
