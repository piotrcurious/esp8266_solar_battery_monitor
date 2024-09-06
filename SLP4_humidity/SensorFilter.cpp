// SensorFilter.cpp
#include "SensorFilter.h"

// Constructor to initialize the SensorFilter object
SensorFilter::SensorFilter(int size, float outlierThreshold, float baseKalmanGain, float processNoise, float measurementNoise) {
    arraySize = (size > 0) ? size : 16;  // Ensure array size is valid
    this->outlierThreshold = (outlierThreshold > 0) ? outlierThreshold : 2.0; // Prevent invalid thresholds
    this->baseKalmanGain = constrainFloat(baseKalmanGain, 0.001, 1.0); // Constrain base Kalman gain to prevent instability
    this->processNoise = (processNoise > 0.0001) ? processNoise : 0.0001; // Ensure positive process noise
    this->measurementNoise = (measurementNoise > 0.0001) ? measurementNoise : 0.0001; // Ensure positive measurement noise
    readings = new float[arraySize];  // Dynamically allocate memory for readings array
    index = 0;
    kalmanGain = this->baseKalmanGain;
    estimate = 0;
    errorCovariance = 1;

    // Initialize readings array with zeros
    for (int i = 0; i < arraySize; i++) {
        readings[i] = 0;
    }
}

// Destructor to free dynamically allocated memory
SensorFilter::~SensorFilter() {
    delete[] readings;
}

// Function to constrain float values safely
float SensorFilter::constrainFloat(float value, float minVal, float maxVal) {
    if (value < minVal) {
        return minVal;
    } else if (value > maxVal) {
        return maxVal;
    }
    return value;
}

// Function to detect if a value is an outlier
bool SensorFilter::isOutlier(float value, float avg, float stdDev) {
    return stdDev > 0 ? abs(value - avg) > outlierThreshold * stdDev : false; // Check to avoid division by zero
}

// Function to calculate the mean of an array
float SensorFilter::calculateMean(float arr[], int size) {
    if (size <= 0) return 0;  // Safety check to prevent division by zero
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size;
}

// Function to calculate the standard deviation of an array
float SensorFilter::calculateStdDev(float arr[], int size, float mean) {
    if (size <= 1) return 0; // Avoid division by zero when size is 1 or less
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += (arr[i] - mean) * (arr[i] - mean);
    }
    return sqrt(sum / size);
}


/* old
// Function to update the Kalman filter
float SensorFilter::kalmanFilter(float measurement, float gain) {
    // Constrain Kalman gain to a safe range to prevent underflow or excessive influence
    gain = constrainFloat(gain, 0.0001, 1.0);
    
    // Update the error covariance and Kalman gain
    errorCovariance += processNoise;
    kalmanGain = gain;

    // Update the estimate
    estimate = estimate + kalmanGain * (measurement - estimate);

    // Adjust the error covariance based on the Kalman gain
    errorCovariance = (1 - kalmanGain) * errorCovariance;

    return estimate;
}
*/

// Function to update the Kalman filter
float SensorFilter::kalmanFilter(float measurement, float gain) {
    // Constrain Kalman gain to a safe range to prevent underflow or excessive influence
    gain = constrainFloat(gain, 0.0001, 1.0);
    measurementNoise = gain  ; // changing conditions
    
    // Prediction update: error covariance is updated by adding process noise
    errorCovariance += processNoise;

    // Calculate Kalman gain using both error covariance and measurement noise
    kalmanGain = errorCovariance / (errorCovariance + measurementNoise);

    // Correction update: update the estimate with the new measurement
    estimate = estimate + kalmanGain * (measurement - estimate);

    // Update the error covariance using the Kalman gain
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
        estimate = kalmanFilter(fallbackAverage, constrainFloat(kalmanGain * 0.1, 0.0001, 0.1));  // lower the measurement noise 
        outlierThreshold = stdDev > 0 ? stdDev : outlierThreshold;  // Update outlier threshold to current stdDev if positive
    } else {
        // If there are valid readings, update the Kalman filter normally and restore Kalman gain
        estimate = kalmanFilter(filteredSum / validCount, baseKalmanGain); // restore the measurement noise 
        //kalmanGain = baseKalmanGain;  // Restore Kalman gain to base value
    }

    // Return the current filtered estimate
    return estimate;
}

// Function to get the current filtered value
float SensorFilter::getFilteredValue() {
    return estimate;
}
