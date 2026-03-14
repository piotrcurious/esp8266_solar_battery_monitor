// SensorFilter.cpp
#include "SensorFilter.h"

SensorFilter::SensorFilter(int size, float outlierThreshold, float processNoise, float measurementNoise)
    : arraySize(size > 0 ? size : 1), outlierThreshold(outlierThreshold), processNoise(processNoise), measurementNoise(measurementNoise) {
    readings = new float[arraySize];
    index = 0;
    count = 0;
    estimate = 0;
    errorCovariance = 1.0f;

    for (int i = 0; i < arraySize; i++) {
        readings[i] = 0;
    }
}

SensorFilter::~SensorFilter() {
    delete[] readings;
}

void SensorFilter::copyFrom(const SensorFilter& other) {
    arraySize = other.arraySize;
    outlierThreshold = other.outlierThreshold;
    processNoise = other.processNoise;
    measurementNoise = other.measurementNoise;
    index = other.index;
    count = other.count;
    estimate = other.estimate;
    errorCovariance = other.errorCovariance;
    readings = new float[arraySize];
    for (int i = 0; i < arraySize; i++) {
        readings[i] = other.readings[i];
    }
}

SensorFilter::SensorFilter(const SensorFilter& other) {
    copyFrom(other);
}

SensorFilter& SensorFilter::operator=(const SensorFilter& other) {
    if (this != &other) {
        delete[] readings;
        copyFrom(other);
    }
    return *this;
}

bool SensorFilter::isOutlier(float value, float avg, float stdDev) const {
    if (stdDev < 1e-6f) return false;
    return abs(value - avg) > outlierThreshold * stdDev;
}

void SensorFilter::calculateStats(float &mean, float &stdDev) const {
    if (count == 0) {
        mean = 0;
        stdDev = 0;
        return;
    }

    float sum = 0.0f;
    float c = 0.0f;
    for (int i = 0; i < count; i++) {
        float y = readings[i] - c;
        float t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }
    mean = sum / count;

    float sqSum = 0.0f;
    c = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = readings[i] - mean;
        float y = (diff * diff) - c;
        float t = sqSum + y;
        c = (t - sqSum) - y;
        sqSum = t;
    }
    stdDev = sqrt(sqSum / count);
}

float SensorFilter::kalmanFilterUpdate(float measurement) {
    errorCovariance += processNoise;
    float kalmanGain = errorCovariance / (errorCovariance + measurementNoise);
    estimate = estimate + kalmanGain * (measurement - estimate);
    errorCovariance = (1.0f - kalmanGain) * errorCovariance;
    return estimate;
}

float SensorFilter::updateSensorReading(float newReading) {
    if (count == 0) {
        estimate = newReading;
        readings[index] = newReading;
        index = (index + 1) % arraySize;
        count = 1;
        return estimate;
    }

    float mean, stdDev;
    calculateStats(mean, stdDev);

    bool outlier = isOutlier(newReading, mean, stdDev);

    readings[index] = newReading;
    index = (index + 1) % arraySize;
    if (count < arraySize) count++;

    if (!outlier) {
        kalmanFilterUpdate(newReading);
    } else {
        errorCovariance += processNoise;
    }

    return estimate;
}
