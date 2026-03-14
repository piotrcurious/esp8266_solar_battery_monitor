// SensorFilter.h
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

/**
 * SensorFilter.h - Optimized Kalman Filter with Outlier Rejection
 * Version: 2.1.0
 *
 * Changelog:
 * 2.1.0: Optimized stats to Welford's algorithm, switched to std::vector.
 * 2.0.0: Initial implementation with Kahan summation and Kalman filter.
 */

#include <vector>
#include <cstddef>

class SensorFilter {
public:
    /**
     * @brief Construct a new Sensor Filter object
     *
     * @param size Number of readings to keep for outlier detection window.
     * @param outlierThreshold Number of standard deviations to trigger rejection.
     * @param processNoise Q parameter for Kalman filter (typically small: 0.001 to 0.1).
     * @param measurementNoise R parameter for Kalman filter (typically 0.1 to 2.0).
     */
    SensorFilter(size_t size, float outlierThreshold = 2.0f, float processNoise = 0.01f, float measurementNoise = 0.1f);
    ~SensorFilter() = default;

    // Rule of Three (defaulted since we use std::vector)
    SensorFilter(const SensorFilter& other) = default;
    SensorFilter& operator=(const SensorFilter& other) = default;

    /**
     * @brief Update the filter with a new sensor reading.
     *
     * @param newReading The raw value from the sensor.
     * @return float The current filtered estimate.
     */
    float updateSensorReading(float newReading);

    /**
     * @brief Get the current filtered value.
     */
    float getFilteredValue() const { return estimate; }

    /**
     * @brief Reset the filter state.
     */
    void reset(float initialValue = 0.0f);

    // Setters for tuning
    void setOutlierThreshold(float t) { outlierThreshold = t; }
    void setProcessNoise(float q) { this->processNoise = q; }
    void setMeasurementNoise(float r) { this->measurementNoise = r; }

private:
    std::vector<float> readings;
    size_t maxSize;
    size_t head;
    size_t count;
    float outlierThreshold;

    // Kalman filter variables
    float estimate;
    float errorCovariance;
    float processNoise;
    float measurementNoise;

    void calculateStats(float &mean, float &stdDev) const;
    void kalmanFilterUpdate(float measurement);
};

#endif
