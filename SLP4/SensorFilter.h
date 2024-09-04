// SensorFilter.h
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <Arduino.h>

class SensorFilter {
public:
    SensorFilter(int size, float outlierThreshold, float baseKalmanGain, float processNoise, float measurementNoise);
    ~SensorFilter(); // Destructor to clean up dynamically allocated memory
    float updateSensorReading(float newReading);  // Function to add a sensor reading, filter outliers, and update the estimate
    float getFilteredValue();  // Function to get the current filtered value

private:
    float *readings;           // Pointer to dynamically allocated array to store sensor readings
    int arraySize;             // Size of the readings array
    float outlierThreshold;    // Threshold for outlier detection
    int index;                 // Current index in the array

    // Kalman filter variables
    float kalmanGain;          // Kalman gain
    float baseKalmanGain;      // Base Kalman gain to restore when readings are normal
    float estimate;            // Estimated sensor value
    float errorCovariance;     // Error covariance of the estimate
    float processNoise;        // Process noise
    float measurementNoise;    // Measurement noise

    bool isOutlier(float value, float avg, float stdDev);    // Function to detect if a value is an outlier
    float calculateMean(float arr[], int size);              // Function to calculate the mean of an array
    float calculateStdDev(float arr[], int size, float mean);// Function to calculate the standard deviation of an array
    float kalmanFilter(float measurement, float gain);       // Function to update the Kalman filter
    float constrainFloat(float value, float minVal, float maxVal); // Function to constrain float values safely
};

#endif
