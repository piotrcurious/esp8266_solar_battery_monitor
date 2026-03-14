#include "SensorFilter.h"
#include <cmath>
#include <algorithm>

SensorFilter::SensorFilter(size_t size, float outlierThreshold, float processNoise, float measurementNoise)
    : readings(size > 0 ? size : 1, 0.0f),
      maxSize(size > 0 ? size : 1),
      head(0),
      count(0),
      outlierThreshold(outlierThreshold),
      consecutiveOutliers(0),
      stepThreshold(5),
      estimate(0.0f),
      errorCovariance(1.0f),
      processNoise(processNoise),
      measurementNoise(measurementNoise),
      innovationVar(measurementNoise) {
}

void SensorFilter::reset(float initialValue) {
    std::fill(readings.begin(), readings.end(), 0.0f);
    head = 0;
    count = 0;
    consecutiveOutliers = 0;
    estimate = initialValue;
    errorCovariance = 1.0f;
    innovationVar = measurementNoise;
}

void SensorFilter::calculateStats(float &mean, float &variance) const {
    if (count == 0) {
        mean = 0.0f;
        variance = 0.0f;
        return;
    }

    // Welford's algorithm for single-pass mean and variance
    float m = 0.0f;
    float s = 0.0f;
    for (size_t i = 0; i < count; ++i) {
        float x = readings[i];
        float nextM = m + (x - m) / (i + 1);
        s += (x - m) * (x - nextM);
        m = nextM;
    }

    mean = m;
    variance = (count > 1) ? (s / count) : 0.0f;
}

void SensorFilter::kalmanFilterUpdate(float measurement) {
    // Prediction
    errorCovariance += processNoise;

    // Innovation (Residual)
    float innovation = measurement - estimate;

    // Adaptive Measurement Noise (Sage-Husa inspired simplified adaptation)
    // We update an estimate of the innovation variance to adapt to changing noise levels
    float alpha = 0.1f; // Adaptation rate
    innovationVar = (1.0f - alpha) * innovationVar + alpha * (innovation * innovation);

    // Ensure adaptiveR doesn't drop too low or explode
    float adaptiveR = std::max(measurementNoise, innovationVar - errorCovariance);

    // Update
    float kalmanGain = errorCovariance / (errorCovariance + adaptiveR);
    estimate = estimate + kalmanGain * innovation;
    errorCovariance = (1.0f - kalmanGain) * errorCovariance;
}

float SensorFilter::updateSensorReading(float newReading) {
    if (count == 0) {
        estimate = newReading;
        readings[head] = newReading;
        head = (head + 1) % maxSize;
        count = 1;
        return estimate;
    }

    float mean, variance;
    calculateStats(mean, variance);

    // Outlier check
    bool outlier = false;
    if (variance > 1e-6f) {
        float diff = newReading - mean;
        if (diff * diff > outlierThreshold * outlierThreshold * variance) {
            outlier = true;
        }
    }

    if (!outlier) {
        consecutiveOutliers = 0;
        kalmanFilterUpdate(newReading);
        // Feed the "clean" reading into the buffer
        readings[head] = newReading;
    } else {
        consecutiveOutliers++;

        // If we see many consecutive outliers, it's likely a signal step, not a spike
        if (consecutiveOutliers >= stepThreshold) {
            estimate = newReading; // Snap to new value
            errorCovariance = measurementNoise; // Reset confidence
            innovationVar = measurementNoise;
            consecutiveOutliers = 0;
            // Fill buffer with new value to stabilize stats quickly
            std::fill(readings.begin(), readings.end(), newReading);
        } else {
            // It's a spike. Don't update Kalman estimate, just age covariance
            errorCovariance += processNoise;
            // Feed the PREVIOUS estimate into the buffer instead of the outlier spike
            // This prevents the spike from poisoning future mean/stdDev calculations
            readings[head] = estimate;
        }
    }

    head = (head + 1) % maxSize;
    if (count < maxSize) count++;

    return estimate;
}
