#include <Arduino.h>

// Structure defining a data field
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
    int bufferSize;          // Total size of the input buffer
    int bufferIndex;         // Current position in the buffer
    int bufferThreshold;     // Threshold at which to trigger parsing when the buffer is almost full

    // Function to remove unwanted characters from a string
    String sanitizeInput(const String& str) {
      String result = "";
      for (int i = 0; i < str.length(); i++) {
        char c = str[i];
        // Allow alphanumeric, decimal points, spaces, and minus signs
        if (isAlphaNumeric(c) || c == '.' || c == ' ' || c == '-') {
          result += c;
        }
      }
      return result;
    }

    // Function to parse the buffer up to the current point and update the target variables
    void processBuffer() {
      inputBuffer[bufferIndex] = '\0';  // Null-terminate the buffer string
      String input = sanitizeInput(String(inputBuffer));  // Clean the input string
      
      // Iterate over fields and extract values based on labels
      for (int i = 0; i < numFields; i++) {
        extractAndUpdateField(input, fields[i]);
      }
    }

    // Function to extract the value of a specific field from the input string
    void extractAndUpdateField(const String& input, DataField& field) {
      String label = String(field.label) + " ";  // Add a space to match labels
      int labelPos = input.indexOf(label);  // Locate the label in the input string

      if (labelPos != -1) {
        int valueStart = labelPos + label.length();  // Position after the label
        String valueStr = input.substring(valueStart).trim();  // Extract and trim the value string

        // Find the next space to determine the end of the number
        int nextSpace = valueStr.indexOf(' ');
        if (nextSpace != -1) {
          valueStr = valueStr.substring(0, nextSpace);
        }

        // Convert the extracted value to a float and update the target variable
        *field.targetVar = valueStr.toFloat();
      }
    }

    // Function to shift buffer contents after partial parsing
    void rollBuffer(int parsedUpTo) {
      if (parsedUpTo > 0) {
        int remainingLength = bufferIndex - parsedUpTo;
        memmove(inputBuffer, inputBuffer + parsedUpTo, remainingLength);  // Shift unparsed data
        bufferIndex = remainingLength;  // Update the buffer index
      }
    }

    // Check if the buffer is close to being full and process it
    void checkAndProcessBufferOnSpace() {
      if (bufferIndex >= bufferThreshold) {  // If buffer is close to being full
        processBuffer();  // Process the buffer up to this point
        rollBuffer(bufferIndex);  // Roll the buffer to make room for new data
      }
    }

  public:
    // Constructor to initialize with a serial port, field array, and buffer size
    SerialParser(Stream& port, DataField* fieldArray, int numFieldElements, int bufSize, int thresholdPercentage = 80)
      : serialPort(port), fields(fieldArray), numFields(numFieldElements), bufferSize(bufSize) {
      inputBuffer = new char[bufferSize];  // Dynamically allocate the buffer
      bufferIndex = 0;  // Initialize the buffer index
      bufferThreshold = (bufferSize * thresholdPercentage) / 100;  // Set the buffer threshold
    }

    // Destructor to free the allocated memory
    ~SerialParser() {
      delete[] inputBuffer;  // Free the dynamically allocated buffer
    }

    // Function to read serial data in a non-blocking manner and process input when needed
    void handleSerialInput() {
      while (serialPort.available() > 0) {
        char incomingChar = serialPort.read();  // Read one character at a time

        // If newline is received, process the buffer and reset it
        if (incomingChar == '\n') {
          processBuffer();  // Process the full line of input
          bufferIndex = 0;  // Reset the buffer index
          continue;
        }

        // Add the character to the input buffer
        inputBuffer[bufferIndex++] = incomingChar;

        // If buffer is close to being full, process up to this point
        if (incomingChar == ' ') {
          checkAndProcessBufferOnSpace();
        }

        // Prevent buffer overflow by resetting the buffer index if it exceeds its size
        if (bufferIndex >= bufferSize) {
          bufferIndex = 0;  // Clear the buffer if it gets too full
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
