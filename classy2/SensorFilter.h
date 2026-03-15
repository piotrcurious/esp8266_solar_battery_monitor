// SensorFilter.h
#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

/**
 * SensorFilter.h - Advanced 2D Kinematic Kalman Filter with Outlier Rejection
 * Version: 3.0.0
 *
 * Changelog:
 * 3.0.0: Upgraded to 2D Kinematic Model (Position/Velocity) with Adaptive Q and R.
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
     * @param size (Deprecated/Size Optimization) Buffer size for windowed fallback (0 to disable buffer).
     * @param outlierThreshold Number of standard deviations to trigger rejection.
     * @param q_pos Process noise for position (default: 0.01).
     * @param q_vel Process noise for velocity (default: 0.001).
     * @param r_noise Measurement noise (default: 0.1).
     */
    SensorFilter(size_t size = 0, float outlierThreshold = 2.0f, float q_pos = 0.01f, float q_vel = 0.001f, float r_noise = 0.1f);
    ~SensorFilter() = default;

    /**
     * @brief Filter state structure for persistence (RTC memory).
     */
    struct State {
        float pos;
        float vel;
        float P00, P01, P11;
        float r_noise;
        float adaptiveQScale;
    };

    /**
     * @brief Update the filter with a new sensor reading.
     */
    float updateSensorReading(float newReading);

    float getFilteredValue() const { return pos; }
    float getVelocity() const { return vel; }
    float getMeasurementNoise() const { return r_noise; }

    void reset(float initialValue = 0.0f);

    /**
     * @brief Persist filter state to external storage (e.g., RTC memory).
     */
    State getState() const;

    /**
     * @brief Restore filter state from external storage.
     */
    void setState(const State& state);

    // Setters for tuning
    void setOutlierThreshold(float t) { outlierThreshold = t; }
    void setProcessNoise(float qp, float qv) { this->q_pos = qp; this->q_vel = qv; }
    void setMeasurementNoise(float r) { this->r_noise = r; }
    void setStepThreshold(size_t s) { this->stepThreshold = s; }

private:
    // Size-optimized storage: Buffer only created if size > 0
    std::vector<float> readings;
    size_t maxSize;
    size_t head;
    size_t count;

    float outlierThreshold;
    size_t consecutiveOutliers;
    size_t stepThreshold;

    // 2D Kalman filter variables (State: [pos, vel])
    float pos;
    float vel;

    // Covariance matrix P
    float P00, P01, P11;

    // Noise parameters
    float q_pos;
    float q_vel;
    float r_noise;

    // Adaptation variables
    float innovationBias;  // Low-pass filtered innovation (bias detection)
    float adaptiveQScale;  // Dynamic Q scaler for agility

    void updateNoiseProfile(float innovation);
};

#endif
