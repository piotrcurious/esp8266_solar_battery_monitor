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

    // Helper: return true for characters allowed inside float tokens
    inline bool isFloatChar(char c) {
      return (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
    }

    // Remove unwanted characters while keeping separators useful for parsing.
    // Replaces other chars with space to preserve token boundaries.
    String sanitizeInput(const String& str) {
      String result;
      result.reserve(str.length());
      for (int i = 0; i < str.length(); ++i) {
        char c = str[i];
        // Keep alphanumeric, decimal point, sign, exponent letters, colon and spaces
        if (isAlphaNumeric(c) || c == '.' || c == '-' || c == '+' || c == ':' || c == ' ' || c == 'e' || c == 'E') {
          result += c;
        } else {
          // preserve token separation
          result += ' ';
        }
      }
      return result;
    }

    // Parse the full buffer and update fields found in it.
    void processBuffer() {
      // ensure null-terminated for String creation
      if (bufferIndex < 0) bufferIndex = 0;
      if (bufferIndex > bufferSize) bufferIndex = bufferSize;
      inputBuffer[bufferIndex] = '\0';

      String raw = String(inputBuffer);
      String s = sanitizeInput(raw);

      // For each registered field attempt to extract and update
      for (int i = 0; i < numFields; ++i) {
        extractAndUpdateField(s, fields[i]);
      }
    }

    // Extract numeric value after label and write it to the target variable.
    // Accepts formats like:
    // "temperature: 23.5", "input voltage 12.3", "output voltage:-4.2e+01"
    void extractAndUpdateField(const String& input, DataField& field) {
      // case-insensitive search — transform copies to lower
      String lowerInput = input;
      lowerInput.toLowerCase();
      String label = String(field.label);
      label.toLowerCase();

      int pos = lowerInput.indexOf(label);
      if (pos == -1) return;

      int idx = pos + label.length();

      // Skip separators like spaces and optional colon(s)
      while (idx < lowerInput.length() && (lowerInput[idx] == ' ' || lowerInput[idx] == ':')) ++idx;
      if (idx >= lowerInput.length()) return;

      // Collect token that looks like a float (allow optional leading sign)
      String token;
      // optional leading sign
      if (idx < lowerInput.length() && (lowerInput[idx] == '+' || lowerInput[idx] == '-')) {
        token += lowerInput[idx++];
      }
      bool hadDigitOrDot = false;
      while (idx < lowerInput.length()) {
        char c = lowerInput[idx];
        if (isFloatChar(c)) {
          token += c;
          if ((c >= '0' && c <= '9') || c == '.') hadDigitOrDot = true;
          ++idx;
        } else {
          break;
        }
      }
      token.trim();
      if (token.length() == 0) return;

      // Convert to float and write back
      *field.targetVar = token.toFloat();
    }

    // Shift unprocessed bytes to the beginning of the buffer.
    // parsedUpTo is the number of bytes that were processed/consumed.
    void rollBuffer(int parsedUpTo) {
      if (parsedUpTo <= 0) return;
      if (parsedUpTo >= bufferIndex) {
        // All consumed
        bufferIndex = 0;
        inputBuffer[0] = '\0';
        return;
      }
      int remaining = bufferIndex - parsedUpTo;
      memmove(inputBuffer, inputBuffer + parsedUpTo, (size_t)remaining);
      bufferIndex = remaining;
      inputBuffer[bufferIndex] = '\0';
    }

    // If buffer is near full, process it to avoid overflow.
    void checkAndProcessBufferOnSpace() {
      if (bufferIndex >= bufferThreshold) {
        processBuffer();
        // after processing, clear buffer to receive fresh data
        bufferIndex = 0;
        inputBuffer[0] = '\0';
      }
    }

  public:
    // Constructor
    SerialParser(Stream& port, DataField* fieldArray, int numFieldElements, int bufSize, int thresholdPercentage = 80)
      : serialPort(port), fields(fieldArray), numFields(numFieldElements), bufferSize(bufSize) {
      // allocate one extra for null terminator
      inputBuffer = new char[bufferSize + 1];
      memset(inputBuffer, 0, (size_t)bufferSize + 1);
      bufferIndex = 0;
      bufferThreshold = max(1, (bufferSize * thresholdPercentage) / 100);
    }

    // Destructor
    ~SerialParser() {
      delete[] inputBuffer;
    }

    // Non-blocking handler — reads available bytes and processes lines or partial content
    void handleSerialInput() {
      while (serialPort.available() > 0) {
        int r = serialPort.read();
        if (r < 0) break;
        char c = (char)r;

        // Normalize CR and treat it as space; LF ends a record
        if (c == '\r') {
          c = ' ';
        }

        if (c == '\n') {
          // full line received — parse what we have
          if (bufferIndex > 0) {
            processBuffer();
            bufferIndex = 0;
            inputBuffer[0] = '\0';
          }
          continue;
        }

        // If adding this char would overflow the buffer, process and reset buffer first
        if (bufferIndex >= bufferSize) {
          // defensive: process what we have and clear
          processBuffer();
          bufferIndex = 0;
          inputBuffer[0] = '\0';
        }

        // Append char
        inputBuffer[bufferIndex++] = c;
        inputBuffer[bufferIndex] = '\0';

        // If this was a space, we may consider processing partial to avoid overflowing with long streams
        if (c == ' ') {
          checkAndProcessBufferOnSpace();
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
#if defined(Serial1)
SerialParser parser2(Serial1, fields2, sizeof(fields2) / sizeof(fields2[0]), 256, 80); // Buffer size 256, threshold 80%
#endif

void setup() {
  Serial.begin(115200);
#if defined(Serial1)
  Serial1.begin(115200);
#endif
  Serial.println("Parser ready. Send lines like: input voltage: 12.34 output voltage: 9.87 temperature: 23.5");
#if defined(Serial1)
  Serial1.println("Parser2 ready on Serial1.");
#endif
}

unsigned long lastPrint = 0;

void loop() {
  // Handle input
  parser1.handleSerialInput();
#if defined(Serial1)
  parser2.handleSerialInput();
#endif

  // Print parsed values at 1 Hz
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("Parser1 - Input Voltage: ");
    Serial.println(inputVoltage1, 4);
    Serial.print("Parser1 - Output Voltage: ");
    Serial.println(outputVoltage1, 4);
    Serial.print("Parser1 - Temperature: ");
    Serial.println(temperature1, 4);

#if defined(Serial1)
    Serial1.print("Parser2 - Input Voltage: ");
    Serial1.println(inputVoltage2, 4);
    Serial1.print("Parser2 - Output Voltage: ");
    Serial1.println(outputVoltage2, 4);
    Serial1.print("Parser2 - Temperature: ");
    Serial1.println(temperature2, 4);
#endif
  }

  // allow other tasks to run
  delay(10);
}
