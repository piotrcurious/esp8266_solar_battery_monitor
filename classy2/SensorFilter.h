// SensorFilter.h
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include <Arduino.h>

class SensorFilter {
public:
    /**
     * @param size Number of readings to keep for outlier detection (rolling window).
     * @param outlierThreshold Number of standard deviations to trigger outlier rejection.
     * @param processNoise Process noise (Q) for Kalman Filter.
     * @param measurementNoise Measurement noise (R) for Kalman Filter.
     */
    SensorFilter(int size, float outlierThreshold, float processNoise, float measurementNoise);
    ~SensorFilter();

    // Rule of Three
    SensorFilter(const SensorFilter& other);
    SensorFilter& operator=(const SensorFilter& other);

    float updateSensorReading(float newReading);
    float getFilteredValue() const { return estimate; }

private:
    float *readings;
    int arraySize;
    float outlierThreshold;
    int index;
    int count;

    // Kalman filter variables
    float estimate;
    float errorCovariance;
    float processNoise;
    float measurementNoise;

    bool isOutlier(float value, float avg, float stdDev) const;
    void calculateStats(float &mean, float &stdDev) const;
    float kalmanFilterUpdate(float measurement);

    void copyFrom(const SensorFilter& other);
};

#endif
