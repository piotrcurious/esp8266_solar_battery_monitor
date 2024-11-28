#include <math.h>

struct MeasurementMonitor {
    float values[4];           // Rolling buffer for the last four measurements
    unsigned long times[4];    // Corresponding timestamps
    int count;                 // Current number of stored samples (max 4)
    float deviationRate;       // Deviation rate between predicted and measured values
    float threshold;           // Threshold for deviation rate
    float shadowValue;         // Shadowed (smoothed) value
    float smoothingFactor;     // Factor for shadowing filter (e.g., 0.2 for 20%)
};

// Initialize the monitor
MeasurementMonitor monitor = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0, 0.0, 0.5, 0.0, 0.2};

// Add a new measurement to the rolling buffer
void addToRollingBuffer(float measurement, unsigned long currentTime, MeasurementMonitor &mon) {
    if (mon.count < 4) {
        mon.values[mon.count] = measurement;
        mon.times[mon.count] = currentTime;
        mon.count++;
    } else {
        for (int i = 1; i < 4; i++) {
            mon.values[i - 1] = mon.values[i];
            mon.times[i - 1] = mon.times[i];
        }
        mon.values[3] = measurement;
        mon.times[3] = currentTime;
    }

    // Apply shadowing filter
    if (mon.count == 1) {
        mon.shadowValue = measurement; // Initialize shadow value on the first sample
    } else {
        mon.shadowValue = mon.smoothingFactor * measurement + (1 - mon.smoothingFactor) * mon.shadowValue;
    }
}

// Calculate the quadratic and cubic regression derivatives
void calculateHigherOrderDerivatives(MeasurementMonitor &mon, float &firstDerivative, float &secondDerivative) {
    if (mon.count < 3) {
        firstDerivative = 0.0;
        secondDerivative = 0.0;
        return;
    }

    float x[4], y[4];
    unsigned long t0 = mon.times[0];
    for (int i = 0; i < mon.count; i++) {
        x[i] = (mon.times[i] - t0) / 1000.0; // Convert timestamps to seconds
        y[i] = mon.values[i];
    }

    // Fit a cubic regression: y = ax^3 + bx^2 + cx + d
    float sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0, sumX5 = 0, sumX6 = 0;
    float sumY = 0, sumXY = 0, sumX2Y = 0, sumX3Y = 0;
    int n = mon.count;

    for (int i = 0; i < n; i++) {
        float xi = x[i];
        float yi = y[i];
        sumX += xi;
        sumX2 += xi * xi;
        sumX3 += xi * xi * xi;
        sumX4 += xi * xi * xi * xi;
        sumX5 += xi * xi * xi * xi * xi;
        sumX6 += xi * xi * xi * xi * xi * xi;
        sumY += yi;
        sumXY += xi * yi;
        sumX2Y += xi * xi * yi;
        sumX3Y += xi * xi * xi * yi;
    }

    // Solve for cubic coefficients using normal equations (reduced to quadratic for simplicity)
    float a = 0.0, b = 0.0, c = 0.0; // Coefficients for ax^2 + bx + c
    float denominator = n * (sumX2 * sumX4 - sumX3 * sumX3) -
                        sumX * (sumX * sumX4 - sumX2 * sumX3) +
                        sumX2 * (sumX * sumX3 - sumX2 * sumX2);
    if (fabs(denominator) > 1e-6) {
        a = (sumY * (sumX2 * sumX4 - sumX3 * sumX3) -
             sumX * (sumXY * sumX4 - sumX2Y * sumX3) +
             sumX2 * (sumXY * sumX3 - sumX2Y * sumX2)) /
            denominator;
        b = (n * (sumXY * sumX4 - sumX2Y * sumX3) -
             sumY * (sumX * sumX4 - sumX2 * sumX3) +
             sumX2 * (sumX * sumX2Y - sumX2 * sumXY)) /
            denominator;
        c = (n * (sumX2 * sumX2Y - sumX3 * sumXY) -
             sumX * (sumX * sumX2Y - sumX2 * sumXY) +
             sumY * (sumX * sumX3 - sumX2 * sumX2)) /
            denominator;
    }

    // Calculate derivatives
    float latestX = x[n - 1];
    firstDerivative = 2 * a * latestX + b;
    secondDerivative = 2 * a;
}

// Predict the time remaining within threshold bounds
float predictTimeWithinBounds(MeasurementMonitor &mon, float firstDerivative, float secondDerivative) {
    if (fabs(firstDerivative) < 1e-6) {
        return INFINITY; // Signal is stable and won't leave bounds
    }

    float currentValue = mon.values[mon.count - 1];
    float upperBound = currentValue * (1.0 + mon.threshold);
    float lowerBound = currentValue * (1.0 - mon.threshold);

    float timeToUpper = (upperBound - currentValue) / firstDerivative;
    float timeToLower = (lowerBound - currentValue) / firstDerivative;

    float predictedTime = INFINITY;
    if (timeToUpper > 0) predictedTime = timeToUpper;
    if (timeToLower > 0 && timeToLower < predictedTime) predictedTime = timeToLower;

    return predictedTime;
}

// Check average change with higher-order derivatives
float checkAverageChange(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addToRollingBuffer(measurement, currentTime, monitor);

    if (monitor.count < 4) {
        timeWithinBounds = INFINITY; // Insufficient data
        return 0.0;
    }

    float firstDerivative, secondDerivative;
    calculateHigherOrderDerivatives(monitor, firstDerivative, secondDerivative);

    float predictedValue = monitor.shadowValue + firstDerivative; // Use shadowed value for prediction
    monitor.deviationRate = fabs(predictedValue - measurement) / fabs(measurement);

    timeWithinBounds = predictTimeWithinBounds(monitor, firstDerivative, secondDerivative);

    return monitor.deviationRate;
}

void setup() {
    Serial.begin(9600);
}

void loop() {
    float measurement = analogRead(A0) * (5.0 / 1023.0); // Convert ADC to voltage
    unsigned long currentTime = millis();

    float timeWithinBounds;
    float deviationRate = checkAverageChange(measurement, currentTime, timeWithinBounds);

    Serial.print("Deviation Rate: ");
    Serial.println(deviationRate);
    Serial.print("Time Within Bounds: ");
    if (isinf(timeWithinBounds)) {
        Serial.println("Infinite (Stable)");
    } else {
        Serial.print(timeWithinBounds);
        Serial.println(" seconds");
    }

    delay(random(50, 150)); // Simulate variable sampling rate
}
