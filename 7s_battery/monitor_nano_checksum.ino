#include <SoftwareSerial.h>

const int analogPins[7] = {A0, A1, A2, A3, A4, A5, A6}; // Analog input pins
const float maxVoltage[7] = {4.2, 8.4, 12.6, 16.8, 21.0, 25.2, 29.4}; // Max voltages per pin (in series cells)
float voltages[7]; // Array to store calculated voltages
float cellVoltages[7]; // Array to store isolated cell voltages

SoftwareSerial mySerial(10, 11); // RX, TX pins for SoftwareSerial (choose available digital pins)

// Function to read analog values and convert to voltages
void readVoltages() {
  for (int i = 0; i < 7; i++) {
    int analogValue = analogRead(analogPins[i]);
    voltages[i] = (analogValue / 1023.0) * maxVoltage[i];
  }
}

// Function to isolate individual cell voltages
void isolateCellVoltages() {
  cellVoltages[0] = voltages[0]; // The first cell voltage is the same as the first stack voltage
  for (int i = 1; i < 7; i++) {
    cellVoltages[i] = voltages[i] - voltages[i - 1];
  }
}

// Function to generate a simple checksum for error correction
int generateChecksum(float *data, int length) {
  int checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= (int)(data[i] * 100); // Multiply by 100 to keep two decimal places
  }
  return checksum;
}

// Function to send data with error correction
void sendCellVoltages() {
  int checksum = generateChecksum(cellVoltages, 7); // Generate checksum for error correction
  mySerial.print("CELL_VOLTAGES:");
  for (int i = 0; i < 7; i++) {
    mySerial.print(cellVoltages[i], 2); // Print cell voltage with 2 decimal places
    if (i < 6) {
      mySerial.print(",");
    }
  }
  mySerial.print(":");
  mySerial.println(checksum); // Print checksum
}

void setup() {
  Serial.begin(9600); // For debugging
  mySerial.begin(1200); // Set baud rate for SoftwareSerial
  Serial.println("Starting Voltage Monitoring...");
}

void loop() {
  readVoltages(); // Read analog inputs and convert to voltages
  isolateCellVoltages(); // Isolate individual cell voltages
  sendCellVoltages(); // Send the isolated voltages with error correction
  delay(1000); // Delay between readings
}
