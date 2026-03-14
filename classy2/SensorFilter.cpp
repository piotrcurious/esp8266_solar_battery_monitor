#include "SensorFilter.h"
#include <cmath>
#include <algorithm>

SensorFilter::SensorFilter(size_t size, float outlierThreshold, float processNoise, float measurementNoise)
    : readings(size > 0 ? size : 1, 0.0f),
      maxSize(size > 0 ? size : 1),
      head(0),
      count(0),
      outlierThreshold(outlierThreshold),
      estimate(0.0f),
      errorCovariance(1.0f),
      processNoise(processNoise),
      measurementNoise(measurementNoise) {
}

void SensorFilter::reset(float initialValue) {
    std::fill(readings.begin(), readings.end(), 0.0f);
    head = 0;
    count = 0;
    estimate = initialValue;
    errorCovariance = 1.0f;
}

void SensorFilter::calculateStats(float &mean, float &stdDev) const {
    if (count == 0) {
        mean = 0.0f;
        stdDev = 0.0f;
        return;
    }

    // Welford's algorithm for single-pass mean and variance
    double m = 0.0;
    double s = 0.0;
    for (size_t i = 0; i < count; ++i) {
        double x = readings[i];
        double nextM = m + (x - m) / (i + 1);
        s += (x - m) * (x - nextM);
        m = nextM;
    }

    mean = static_cast<float>(m);
    stdDev = (count > 1) ? static_cast<float>(std::sqrt(s / count)) : 0.0f;
}

void SensorFilter::kalmanFilterUpdate(float measurement) {
    // Prediction
    errorCovariance += processNoise;

    // Update
    float kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
    estimate = estimate + kalmanGain * (measurement - estimate);
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

    float mean, stdDev;
    calculateStats(mean, stdDev);

    // Outlier check
    bool outlier = false;
    if (stdDev > 1e-6f) {
        if (std::abs(newReading - mean) > outlierThreshold * stdDev) {
            outlier = true;
        }
    }

    // Update rolling window
    readings[head] = newReading;
    head = (head + 1) % maxSize;
    if (count < maxSize) count++;

    if (!outlier) {
        kalmanFilterUpdate(newReading);
    } else {
        // If outlier, we still "age" the covariance to allow for gradual recovery
        errorCovariance += processNoise;
    }

    return estimate;
}
