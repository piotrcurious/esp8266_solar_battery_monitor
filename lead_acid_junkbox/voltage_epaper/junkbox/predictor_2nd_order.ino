#include <math.h> // For mathematical functions

struct MeasurementMonitor {
    float values[4];           // Rolling buffer for the last four measurements
    unsigned long times[4];    // Corresponding timestamps
    int count;                 // Current number of stored samples (max 4)
    float deviationRate;       // Deviation rate between predicted and measured values
    float threshold;           // Threshold for deviation rate
};

// Initialize the monitor
MeasurementMonitor monitor = {{0, 0, 0, 0}, {0, 0, 0, 0}, 0, 0.0, 0.5};

// Function to add a new measurement to the rolling buffer
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
}

// Function to calculate the nonlinear derivative using quadratic regression
float calculateNonlinearDerivative(MeasurementMonitor &mon) {
    if (mon.count < 3) {
        return 0.0; // Not enough data for quadratic regression
    }

    float x[4], y[4];
    unsigned long t0 = mon.times[0];
    for (int i = 0; i < mon.count; i++) {
        x[i] = (mon.times[i] - t0) / 1000.0; // Convert timestamps to seconds relative to t0
        y[i] = mon.values[i];
    }

    // Fit a quadratic regression: y = ax^2 + bx + c
    float sumX = 0, sumX2 = 0, sumX3 = 0, sumX4 = 0;
    float sumY = 0, sumXY = 0, sumX2Y = 0;
    int n = mon.count;

    for (int i = 0; i < n; i++) {
        float xi = x[i];
        float yi = y[i];
        sumX += xi;
        sumX2 += xi * xi;
        sumX3 += xi * xi * xi;
        sumX4 += xi * xi * xi * xi;
        sumY += yi;
        sumXY += xi * yi;
        sumX2Y += xi * xi * yi;
    }

    // Solve the system of equations for a, b, and c
    float denominator = n * (sumX2 * sumX4 - sumX3 * sumX3) -
                        sumX * (sumX * sumX4 - sumX2 * sumX3) +
                        sumX2 * (sumX * sumX3 - sumX2 * sumX2);
    if (fabs(denominator) < 1e-6) {
        return 0.0; // Ill-conditioned matrix, fallback to zero derivative
    }

    float a = (sumY * (sumX2 * sumX4 - sumX3 * sumX3) -
               sumX * (sumXY * sumX4 - sumX2Y * sumX3) +
               sumX2 * (sumXY * sumX3 - sumX2Y * sumX2)) /
              denominator;
    float b = (n * (sumXY * sumX4 - sumX2Y * sumX3) -
               sumY * (sumX * sumX4 - sumX2 * sumX3) +
               sumX2 * (sumX * sumX2Y - sumX2 * sumXY)) /
              denominator;
    float c = (n * (sumX2 * sumX2Y - sumX3 * sumXY) -
               sumX * (sumX * sumX2Y - sumX2 * sumXY) +
               sumY * (sumX * sumX3 - sumX2 * sumX2)) /
              denominator;

    // Calculate the derivative at the most recent timestamp
    float latestX = x[n - 1];
    return 2 * a * latestX + b;
}

// Function to predict the next value based on the derivative
float predictNextValue(MeasurementMonitor &mon, float measuredDerivative, unsigned long currentTime) {
    float deltaTime = (currentTime - mon.times[mon.count - 1]) / 1000.0;
    return mon.values[mon.count - 1] + measuredDerivative * deltaTime;
}

// Function to predict the time remaining before the signal exits the threshold bounds
float predictTimeWithinBounds(MeasurementMonitor &mon, float measuredDerivative) {
    if (abs(measuredDerivative) < 1e-6) {
        return INFINITY; // Signal is stable and won't leave bounds
    }

    float currentValue = mon.values[mon.count - 1];
    float upperBound = currentValue * (1.0 + mon.threshold);
    float lowerBound = currentValue * (1.0 - mon.threshold);

    float timeToUpper = (upperBound - currentValue) / measuredDerivative;
    float timeToLower = (lowerBound - currentValue) / measuredDerivative;

    float predictedTime = INFINITY;
    if (timeToUpper > 0) predictedTime = timeToUpper;
    if (timeToLower > 0 && timeToLower < predictedTime) predictedTime = timeToLower;

    return predictedTime;
}

// Function to check average change and calculate deviation rate
float checkAverageChange(float measurement, unsigned long currentTime, float &timeWithinBounds) {
    addToRollingBuffer(measurement, currentTime, monitor);

    if (monitor.count < 4) {
        timeWithinBounds = INFINITY; // Insufficient data
        return 0.0;
    }

    float measuredDerivative = calculateNonlinearDerivative(monitor);
    float predictedValue = predictNextValue(monitor, measuredDerivative, currentTime);
    monitor.deviationRate = abs(predictedValue - measurement) / abs(measurement);

    // Predict how long the signal will stay within the threshold bounds
    timeWithinBounds = predictTimeWithinBounds(monitor, measuredDerivative);

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

    if (deviationRate > monitor.threshold) {
        Serial.println("Deviation rate exceeded the threshold!");
    }

    delay(random(50, 150)); // Simulate variable sampling rate
}
