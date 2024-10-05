#include <Arduino.h>

// Structure defining a field
struct DataField {
  const char* label;    // Label identifying the data field in the input
  float* targetVar;     // Pointer to the target variable to store the extracted value
};

class SerialParser {
  private:
    Stream& serialPort;      // Reference to the serial port (e.g., Serial, Serial1)
    DataField* fields;       // Array of fields to parse
    int numFields;           // Number of fields
    char* inputBuffer;       // Buffer to accumulate incoming serial data
    int bufferSize;          // Size of the input buffer
    int bufferIndex;         // Current position in the buffer
    int bufferThreshold;     // Threshold at which to trigger parsing when the buffer is almost full

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

    // Function to roll the buffer after parsing, removing already parsed part
    void rollBuffer(int parsedUpTo) {
      if (parsedUpTo > 0) {
        // Shift remaining data to the beginning of the buffer
        int remainingLength = bufferIndex - parsedUpTo;
        memmove(inputBuffer, inputBuffer + parsedUpTo, remainingLength);
        bufferIndex = remainingLength;  // Update the buffer index to the new position
      }
    }

  public:
    // Constructor to initialize with a serial port, field array, and buffer size
    SerialParser(Stream& port, DataField* fieldArray, int numFieldElements, int bufferSize, int thresholdPercentage = 80)
      : serialPort(port), fields(fieldArray), numFields(numFieldElements), bufferSize(bufferSize) {
      inputBuffer = new char[bufferSize];  // Allocate buffer dynamically
      bufferIndex = 0;  // Initialize buffer index
      bufferThreshold = (bufferSize * thresholdPercentage) / 100;  // Set buffer threshold
    }

    // Destructor to free the allocated memory
    ~SerialParser() {
      delete[] inputBuffer;  // Free the dynamically allocated buffer
    }

    // Function to read serial data non-blocking and process when a newline or near-full threshold is detected
    void handleSerialInput() {
      while (serialPort.available() > 0) {
        char incomingChar = serialPort.read();  // Read one character at a time

        // If the buffer is close to being full, trigger parsing
        if (bufferIndex >= bufferThreshold && incomingChar == ' ') {
          inputBuffer[bufferIndex] = '\0';  // Null-terminate the string before parsing
          parseSerialInput(String(inputBuffer));  // Process the buffered input up to this point
          rollBuffer(bufferIndex);  // Roll the buffer after parsing
        }

        // If the buffer is full, clear it to avoid overflow
        if (bufferIndex >= bufferSize - 1) {
          bufferIndex = 0;  // Reset buffer index
        }

        // Add the character to the input buffer
        inputBuffer[bufferIndex++] = incomingChar;

        // If newline is received, process the input buffer
        if (incomingChar == '\n') {
          inputBuffer[bufferIndex - 1] = '\0';  // Null-terminate the string before parsing
          parseSerialInput(String(inputBuffer));  // Process the complete line of input
          bufferIndex = 0;  // Clear the buffer for the next input
        }
      }
    }
};

// Target variables for the first parser instance
float inputVoltage1 = 0;
float outputVoltage1 = 0;
float temperature1 = 0;

// Target variables for the second parser instance
float inputVoltage2 = 0;
float outputVoltage2 = 0;
float temperature2 = 0;

// Define the fields for the first parser
DataField fields1[] = {
  {"input voltage", &inputVoltage1},
  {"output voltage", &outputVoltage1},
  {"temperature", &temperature1}
};

// Define the fields for the second parser
DataField fields2[] = {
  {"input voltage", &inputVoltage2},
  {"output voltage", &outputVoltage2},
  {"temperature", &temperature2}
};

// Create instances of SerialParser for different serial ports with different buffer sizes
SerialParser parser1(Serial, fields1, sizeof(fields1) / sizeof(fields1[0]), 128, 80);  // Buffer size 128, threshold 80%
SerialParser parser2(Serial1, fields2, sizeof(fields2) / sizeof(fields2[0]), 256, 80); // Buffer size 256, threshold 80%

void setup() {
  Serial.begin(115200);   // Initialize Serial for the first parser
  Serial1.begin(115200);  // Initialize Serial1 for the second parser
  Serial.println("Enter data for parser 1:");
  Serial1.println("Enter data for parser 2:");
}

void loop() {
  // Non-blocking serial input handling for both parser instances
  parser1.handleSerialInput();
  parser2.handleSerialInput();

  // Output the parsed values for the first parser instance
  Serial.print("Parser1 - Input Voltage: ");
  Serial.println(inputVoltage1);
  Serial.print("Parser1 - Output Voltage: ");
  Serial.println(outputVoltage1);
  Serial.print("Parser1 - Temperature: ");
  Serial.println(temperature1);

  // Output the parsed values for the second parser instance
  Serial1.print("Parser2 - Input Voltage: ");
  Serial1.println(inputVoltage2);
  Serial1.print("Parser2 - Output Voltage: ");
  Serial1.println(outputVoltage2);
  Serial1.print("Parser2 - Temperature: ");
  Serial1.println(temperature2);

  delay(1000);  // Simulating other tasks running
}
