// SensorFilter.cpp
#include "SensorFilter.h"

// Constructor to initialize the SensorFilter object
SensorFilter::SensorFilter(int size, float outlierThreshold, float baseKalmanGain, float processNoise, float measurementNoise) {
    arraySize = size;
    this->outlierThreshold = outlierThreshold;
    this->baseKalmanGain = baseKalmanGain;
    this->processNoise = processNoise;
    this->measurementNoise = measurementNoise;
    readings = new float[arraySize];  // Dynamically allocate memory for readings array
    index = 0;
    kalmanGain = baseKalmanGain;
    estimate = 0;
    errorCovariance = 1;

    // Initialize readings array with zeros
    for (int i = 0; i < arraySize; i++) {
        readings[i] = 0;
    }
}

// Function to detect if a value is an outlier
bool SensorFilter::isOutlier(float value, float avg, float stdDev) {
    return abs(value - avg) > outlierThreshold * stdDev;
}

// Function to calculate the mean of an array
float SensorFilter::calculateMean(float arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size;
}

// Function to calculate the standard deviation of an array
float SensorFilter::calculateStdDev(float arr[], int size, float mean) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += (arr[i] - mean) * (arr[i] - mean);
    }
    return sqrt(sum / size);
}

// Function to update the Kalman filter
float SensorFilter::kalmanFilter(float measurement) {
    // Update the error covariance and Kalman gain
    errorCovariance += processNoise;
    kalmanGain = errorCovariance / (errorCovariance + measurementNoise);

    // Update the estimate
    estimate = estimate + kalmanGain * (measurement - estimate);

    // Adjust the error covariance based on the Kalman gain
    errorCovariance = (1 - kalmanGain) * errorCovariance;

    return estimate;
}

// Function to add a sensor reading, filter outliers, and update the estimate
float SensorFilter::updateSensorReading(float newReading) {
    // Add the new reading to the array
    readings[index] = newReading;
    index = (index + 1) % arraySize;

    // Calculate the mean and standard deviation of the readings
    float mean = calculateMean(readings, arraySize);
    float stdDev = calculateStdDev(readings, arraySize, mean);

    // Check if the new reading is an outlier
    if (!isOutlier(newReading, mean, stdDev)) {
        // Update the Kalman filter if the reading is not an outlier
        estimate = kalmanFilter(newReading);

        // Restore Kalman gain to its base value
        kalmanGain = baseKalmanGain;
    } else {
        // Reduce Kalman gain to minimize impact of outlier
        kalmanGain *= 0.5;
    }

    // Calculate filtered mean, excluding outliers
    float filteredSum = 0;
    int count = 0;
    for (int i = 0; i < arraySize; i++) {
        if (!isOutlier(readings[i], mean, stdDev)) {
            filteredSum += readings[i];
            count++;
        }
    }

    // Return the rolling average of non-outlier readings or the estimate if no valid readings
    return (count > 0) ? filteredSum / count : estimate;
}

// Function to get the current filtered value
float SensorFilter::getFilteredValue() {
    return estimate;
}
