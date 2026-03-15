#include "SensorFilter.h"
#include <cmath>
#include <algorithm>

SensorFilter::SensorFilter(size_t size, float outlierThreshold, float q_pos, float q_vel, float r_noise)
    : maxSize(size), head(0), count(0),
      outlierThreshold(outlierThreshold), consecutiveOutliers(0), stepThreshold(5),
      pos(0.0f), vel(0.0f),
      P00(1.0f), P01(0.0f), P11(1.0f),
      q_pos(q_pos), q_vel(q_vel), r_noise(r_noise),
      innovationBias(0.0f), adaptiveQScale(1.0f)
{
    if (maxSize > 0) {
        readings.resize(maxSize, 0.0f);
    }
}

void SensorFilter::reset(float initialValue) {
    pos = initialValue;
    vel = 0.0f;
    P00 = 1.0f;
    P01 = 0.0f;
    P11 = 1.0f;
    count = 0; head = 0;
    consecutiveOutliers = 0;
    innovationBias = 0.0f;
    adaptiveQScale = 1.0f;
    if (!readings.empty()) {
        std::fill(readings.begin(), readings.end(), 0.0f);
    }
}

SensorFilter::State SensorFilter::getState() const {
    return {pos, vel, P00, P01, P11, r_noise, adaptiveQScale};
}

void SensorFilter::setState(const State& state) {
    pos = state.pos;
    vel = state.vel;
    P00 = state.P00;
    P01 = state.P01;
    P11 = state.P11;
    r_noise = state.r_noise;
    adaptiveQScale = state.adaptiveQScale;
}

float SensorFilter::updateSensorReading(float newReading) {
    if (count == 0) {
        pos = newReading;
        if (!readings.empty()) {
            readings[head] = newReading;
            head = (head + 1) % maxSize;
        }
        count = 1;
        return pos;
    }

    // 1. Prediction (Kinematic Model)
    float damping = 0.98f;
    pos = pos + vel;
    vel = vel * damping;

    // Uncertainty propagation P = F*P*F' + Q
    float qScale = adaptiveQScale;
    float nP00 = P00 + 2.0f*P01 + P11 + q_pos * qScale;
    float nP01 = damping*(P01 + P11);
    float nP11 = damping*damping*P11 + q_vel * qScale;

    P00 = nP00; P01 = nP01; P11 = nP11;

    // 2. Outlier Rejection
    float innovation = newReading - pos;
    float S = P00 + r_noise;
    float innovSq = innovation * innovation;

    // Robust threshold
    float thresholdSq = (outlierThreshold * outlierThreshold) * std::max(S, r_noise);

    if (innovSq > thresholdSq) {
        consecutiveOutliers++;
        if (consecutiveOutliers < stepThreshold) {
            // Outlier: Advance buffer with prediction to keep window moving
            if (!readings.empty()) {
                readings[head] = pos;
                head = (head + 1) % maxSize;
            }
            return pos;
        }
        // Step detected: Snap to new value
        P00 += innovSq;
        P11 += q_vel * 100.0f;
        vel = 0.0f; // Kill momentum to prevent overshoot after burst noise
    }
    consecutiveOutliers = 0;

    // 3. Kalman Gain
    S = P00 + r_noise;
    float K0 = P00 / S;
    float K1 = P01 / S;

    // 4. Update State
    pos = pos + K0 * innovation;
    vel = vel + K1 * innovation;

    // 5. Update Covariance
    float oldP01 = P01;
    P00 = (1.0f - K0) * P00;
    P01 = (1.0f - K0) * P01;
    P11 = P11 - K1 * oldP01;

    // Numerical Stability
    if (P00 < 1e-6f) P00 = 1e-6f;
    if (P11 < 1e-7f) P11 = 1e-7f;

    // 6. Update Buffer
    if (!readings.empty()) {
        readings[head] = newReading;
        head = (head + 1) % maxSize;
        if (count < maxSize) count++;
    }

    // 7. Adaptation
    updateNoiseProfile(innovation);

    return pos;
}

void SensorFilter::updateNoiseProfile(float innovation) {
    // Sage-Husa measurement noise estimation
    float alpha = 0.05f;
    float r_est = (innovation * innovation) - P00;

    // Window-based floor for R if buffer exists (Kahan Summation logic)
    float r_floor = 0.001f;
    if (count >= maxSize && maxSize > 0) {
        float sum = 0, c = 0;
        for(float r : readings) {
            float y = r - pos - c;
            float t = sum + y;
            c = (t - sum) - y;
            sum = t;
        }
        float mean_diff = sum / count;
        float var_sum = 0; c = 0;
        for(float r : readings) {
            float diff = (r - pos) - mean_diff;
            float y = (diff * diff) - c;
            float t = var_sum + y;
            c = (t - var_sum) - y;
            var_sum = t;
        }
        r_floor = std::max(r_floor, var_sum / count);
    }

    if (r_est < r_floor) r_est = r_floor;
    r_noise = (1.0f - alpha) * r_noise + alpha * r_est;

    // Adaptive Q: Maneuver detection
    float biasAlpha = 0.1f;
    innovationBias = (1.0f - biasAlpha) * innovationBias + biasAlpha * innovation;

    float biasMetric = (innovationBias * innovationBias) / (r_noise + 1e-9f);
    if (biasMetric > 3.0f) { // More conservative threshold
        adaptiveQScale = std::min(adaptiveQScale * 1.1f, 50.0f);
    } else {
        adaptiveQScale = std::max(adaptiveQScale * 0.9f, 1.0f);
    }
}
