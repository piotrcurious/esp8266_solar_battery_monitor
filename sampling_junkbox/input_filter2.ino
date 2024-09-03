// Define the constants
const int ARRAY_SIZE = 16;
const float OUTLIER_THRESHOLD = 2.0;  // Threshold for outlier detection (adjust based on noise characteristics)
float readings[ARRAY_SIZE];           // Array to store sensor readings
int index = 0;                        // Current index in the array

// Kalman filter variables
float kalmanGain = 0.1;               // Initial Kalman gain
float baseKalmanGain = 0.1;           // Base Kalman gain to restore to when readings are normal
float estimate = 0;                   // Estimated sensor value
float errorCovariance = 1;            // Error covariance of the estimate
float processNoise = 0.1;             // Process noise (adjust based on sensor characteristics)
float measurementNoise = 1;           // Measurement noise

// Function to detect if a value is an outlier
bool isOutlier(float value, float avg, float stdDev) {
  return abs(value - avg) > OUTLIER_THRESHOLD * stdDev;
}

// Function to calculate the mean of an array
float calculateMean(float arr[], int size) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += arr[i];
  }
  return sum / size;
}

// Function to calculate the standard deviation of an array
float calculateStdDev(float arr[], int size, float mean) {
  float sum = 0;
  for (int i = 0; i < size; i++) {
    sum += (arr[i] - mean) * (arr[i] - mean);
  }
  return sqrt(sum / size);
}

// Function to update the Kalman filter
float kalmanFilter(float measurement) {
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
float updateSensorReading(float newReading) {
  // Add the new reading to the array
  readings[index] = newReading;
  index = (index + 1) % ARRAY_SIZE;

  // Calculate the mean and standard deviation of the readings
  float mean = calculateMean(readings, ARRAY_SIZE);
  float stdDev = calculateStdDev(readings, ARRAY_SIZE, mean);

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
  for (int i = 0; i < ARRAY_SIZE; i++) {
    if (!isOutlier(readings[i], mean, stdDev)) {
      filteredSum += readings[i];
      count++;
    }
  }

  // Return the rolling average of non-outlier readings
  return (count > 0) ? filteredSum / count : estimate;
}

void setup() {
  Serial.begin(9600);
  // Initialize readings array with zeros
  for (int i = 0; i < ARRAY_SIZE; i++) {
    readings[i] = 0;
  }
}

void loop() {
  // Simulate sensor reading
  float sensorValue = analogRead(A0); // Replace with your sensor reading method

  // Scale sensor reading for demonstration (e.g., 0 to 5V)
  sensorValue = (sensorValue / 1023.0) * 5.0;

  // Update sensor reading with filtering
  float filteredValue = updateSensorReading(sensorValue);

  // Print the filtered sensor value
  Serial.println(filteredValue);

  delay(100); // Delay to simulate sensor update rate
}
