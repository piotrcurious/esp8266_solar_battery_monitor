#include <Arduino.h>

// Structure defining a field
struct DataField {
  const char* label;    // Label identifying the data field in the input
  float* targetVar;     // Pointer to the target variable to store the extracted value
};

// Target variables
float inputVoltage = 0;
float outputVoltage = 0;
float temperature = 0;

// Define the fields we want to parse
DataField fields[] = {
  {"input voltage", &inputVoltage},
  {"output voltage", &outputVoltage},
  {"temperature", &temperature}
};

// Number of fields to parse
const int numFields = sizeof(fields) / sizeof(fields[0]);

// Buffer to store the incoming serial data
String inputBuffer = "";

// Function to remove extra spaces and non-alphanumeric characters from a string
String sanitizeInput(String str) {
  String result = "";
  for (int i = 0; i < str.length(); i++) {
    char c = str[i];
    if (isAlphaNumeric(c) || c == '.' || c == ' ' || c == '-') {  // Allow alphanumeric, decimal points, spaces, and minus signs
      result += c;
    }
  }
  return result;
}

// Function to remove extra spaces from a string
String trimSpaces(String str) {
  str.trim(); // Removes spaces from both ends
  return str;
}

// Function to parse serial input and extract numeric data
void parseSerialInput(String input) {
  input = sanitizeInput(input);  // Remove commas, colons, and other unwanted characters
  input = trimSpaces(input);     // Remove unnecessary spaces

  for (int i = 0; i < numFields; i++) {
    String label = String(fields[i].label) + " ";  // Label with a trailing space for proper matching
    int labelIndex = input.indexOf(label);         // Find label in input

    if (labelIndex != -1) {
      // Find the position right after the label
      int valueStartIndex = labelIndex + label.length();

      // Extract the substring after the label
      String valueStr = input.substring(valueStartIndex);

      // Trim the value string to remove any excess spaces
      valueStr = trimSpaces(valueStr);

      // Find the next space or non-numeric character to terminate the number
      int nextSpaceIndex = valueStr.indexOf(' ');
      if (nextSpaceIndex != -1) {
        valueStr = valueStr.substring(0, nextSpaceIndex);
      }

      // Convert the extracted substring to a float and update the target variable
      float value = valueStr.toFloat();
      *fields[i].targetVar = value;  // Update the target variable
    }
  }
}

// Function to read serial data non-blocking and process when a newline is received
void handleSerialInput() {
  while (Serial.available() > 0) {
    char incomingChar = Serial.read();  // Read one character at a time

    // Add the character to the input buffer
    inputBuffer += incomingChar;

    // If newline is received, process the input buffer
    if (incomingChar == '\n') {
      inputBuffer.trim();  // Remove any excess whitespace
      parseSerialInput(inputBuffer);  // Process the complete line of input
      inputBuffer = "";  // Clear the buffer for the next input
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Enter data in any order, format: 'input voltage: X output voltage: Y temperature: Z'");
}

void loop() {
  // Non-blocking serial input handler
  handleSerialInput();

  // Output the parsed values (you can add conditions to print when values change)
  Serial.print("Input Voltage: ");
  Serial.println(inputVoltage);
  Serial.print("Output Voltage: ");
  Serial.println(outputVoltage);
  Serial.print("Temperature: ");
  Serial.println(temperature);

  // Perform other non-blocking tasks here
  delay(1000);  // Simulating some other task running at intervals
}
