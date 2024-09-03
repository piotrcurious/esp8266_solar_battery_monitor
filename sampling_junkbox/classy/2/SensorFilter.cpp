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
float SensorFilter::kalmanFilter(float measurement, float gain) {
    // Update the error covariance and Kalman gain
    errorCovariance += processNoise;
    kalmanGain = gain;  // Use the provided gain instead of recalculating

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

    // Variables to track the sum of valid readings and their count
    float filteredSum = 0;
    int validCount = 0;

    // Iterate over readings to calculate filtered sum and count of non-outliers
    for (int i = 0; i < arraySize; i++) {
        if (!isOutlier(readings[i], mean, stdDev)) {
            filteredSum += readings[i];
            validCount++;
        }
    }

    // Check if all readings are outliers
    if (validCount == 0) {
        // If all readings are outliers, compute average of all readings and adjust Kalman gain
        float fallbackAverage = calculateMean(readings, arraySize);
        estimate = kalmanFilter(fallbackAverage, kalmanGain * 0.1);  // Use low Kalman gain
        outlierThreshold = stdDev;  // Update outlier threshold to current stdDev to prevent lockup
    } else {
        // If there are valid readings, update the Kalman filter normally and restore Kalman gain
        estimate = kalmanFilter(filteredSum / validCount, baseKalmanGain);
        kalmanGain = baseKalmanGain;  // Restore Kalman gain to base value
    }

    // Return the current filtered estimate
    return estimate;
}

// Function to get the current filtered value
float SensorFilter::getFilteredValue() {
    return estimate;
}
