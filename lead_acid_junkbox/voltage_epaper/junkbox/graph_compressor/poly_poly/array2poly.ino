#include <Arduino.h>
#include <Eigen.h>

using namespace Eigen;

// Structure to hold timestamp delta and value pairs
struct DataPoint {
  unsigned long timestampDelta;
  float value;
};

// Function to fit a 4th order polynomial to data
VectorXd fitPolynomial(DataPoint data[], int numPoints) {
  if (numPoints < 5) { // Need at least 5 points for a 4th order fit
    Serial.println("Error: Insufficient data points for 4th order fit.");
    return VectorXd::Zero(5); // Return a zero vector to indicate error
  }

  MatrixXd X(numPoints, 5);
  VectorXd y(numPoints);

  for (int i = 0; i < numPoints; i++) {
    float xVal = (float)data[i].timestampDelta; // Use timestamp delta as x
    X(i, 0) = 1.0;
    X(i, 1) = xVal;
    X(i, 2) = xVal * xVal;
    X(i, 3) = xVal * xVal * xVal;
    X(i, 4) = xVal * xVal * xVal * xVal;
    y(i) = data[i].value;
  }

  // Solve for coefficients using least squares (normal equation)
  VectorXd coefficients = (X.transpose() * X).inverse() * X.transpose() * y;

  return coefficients;
}

// Example usage
const int BUFFER_SIZE = 20; // Example buffer size
DataPoint dataBuffer[BUFFER_SIZE];
int dataCount = 0;


void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0)); // Seed the random number generator

    //Generate test data
    for(int i=0; i< BUFFER_SIZE; i++){
        dataBuffer[i].timestampDelta = i * 100; // Example timestamp deltas
        dataBuffer[i].value = 0.1*pow(i,4) - 2*pow(i,3) + 5*pow(i,2) - 3*i + 10 + random(-5,5); // Example values with noise
        dataCount++;
    }
}

void loop() {
  // Example usage: Fit polynomial every 5 seconds (or based on some trigger)
  static unsigned long lastFitTime = 0;
  if (millis() - lastFitTime >= 5000) {
    lastFitTime = millis();

    if(dataCount >= 5){
        VectorXd coeffs = fitPolynomial(dataBuffer, dataCount);

        if (coeffs.size() == 5) {
          Serial.println("Polynomial Coefficients:");
          for (int i = 0; i < 5; i++) {
            Serial.print("a");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(coeffs(i), 6); // Print with 6 decimal places
          }

          // Example: Calculate the fitted value at a specific timestamp delta
          float testX = 1500;
          float fittedValue = coeffs(0) + coeffs(1) * testX + coeffs(2) * pow(testX, 2) + coeffs(3) * pow(testX, 3) + coeffs(4) * pow(testX, 4);
          Serial.print("Fitted value at x = ");
          Serial.print(testX);
          Serial.print(": ");
          Serial.println(fittedValue, 6);
        }
    } else {
        Serial.println("Not enough data to perform fit yet");
    }
  }

  // In a real application, you would be continuously adding data to the buffer here.
  // Example:
  // if(newDataAvailable()){
  //   dataBuffer[dataCount % BUFFER_SIZE] = getNewData();
  //   dataCount++;
  // }

  delay(100);
}
