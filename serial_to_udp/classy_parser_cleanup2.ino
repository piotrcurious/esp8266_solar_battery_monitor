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

    // Allow characters that are useful when extracting floats (digits, sign, decimal point, exponent)
    inline bool isFloatChar(char c) {
      return (c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E';
    }

    // Function to remove mostly-unwanted characters while keeping label text and numbers.
    // We keep spaces and colons so label/value separation remains obvious.
    String sanitizeInput(const String& str) {
      String result;
      result.reserve(str.length());
      for (int i = 0; i < str.length(); i++) {
        char c = str[i];
        // Allow letters, digits, decimal points, spaces, minus/plus, colon and exponent letters
        if (isAlphaNumeric(c) || c == '.' || c == ' ' || c == '-' || c == '+' || c == ':' || c == 'e' || c == 'E') {
          result += c;
        } else {
          // replace other separators with space to keep tokens separated
          result += ' ';
        }
      }
      return result;
    }

    // Extract the numeric token following the label (case-insensitive).
    // Returns true if a value was found and updated.
    bool extractAndUpdateField(const String& inputRaw, DataField& field) {
      // Work with lowercase copies for case-insensitive search
      String input = inputRaw;
      input.toLowerCase();
      String label = String(field.label);
      label.toLowerCase();

      // Accept label followed by optional colon and/or spaces
      int pos = input.indexOf(label);
      if (pos == -1) return false;

      int afterLabel = pos + label.length();

      // Skip any colons and spaces
      while (afterLabel < input.length() && (input[afterLabel] == ':' || input[afterLabel] == ' ')) {
        afterLabel++;
      }

      if (afterLabel >= input.length()) return false;

      // Build the numeric token from allowed float characters
      String token;
      int i = afterLabel;
      // allow an initial + or - if present
      if (i < input.length() && (input[i] == '+' || input[i] == '-')) {
        token += input[i++];
      }
      bool seenDigitOrDot = false;
      while (i < input.length()) {
        char c = input[i];
        if (isFloatChar(c)) {
          token += c;
          if ((c >= '0' && c <= '9') || c == '.') seenDigitOrDot = true;
          i++;
        } else {
          break;
        }
      }

      // If token is empty or invalid, bail out
      token.trim();
      if (token.length() == 0) return false;
      // Convert token to float and store
      *field.targetVar = token.toFloat();
      return true;
    }

    // Process the full buffer content (up to bufferIndex).
    // This updates the target variables for all fields found in the input.
    void processBuffer() {
      // ensure null-terminated
      inputBuffer[bufferIndex] = '\0';
      String rawInput = String(inputBuffer);
      // sanitize but keep separators that help parsing
      String clean = sanitizeInput(rawInput);

      // For each field attempt to extract and update
      for (int i = 0; i < numFields; i++) {
        extractAndUpdateField(clean, fields[i]);
      }
    }

    // Reset the buffer (clear contents) safely
    void resetBuffer() {
      if (bufferIndex > 0) {
        // zero only the used portion for a small speed advantage
        memset(inputBuffer, 0, (size_t)bufferIndex + 1);
      }
      bufferIndex = 0;
    }

    // Called when buffer is near full (process and clear)
    void checkAndProcessBufferOnSpace() {
      if (bufferIndex >= bufferThreshold) {
        processBuffer();
        resetBuffer();
      }
    }

  public:
    // Constructor to initialize with a serial port, field array, and buffer size
    // thresholdPercentage defaults to 80% of bufferSize
    SerialParser(Stream& port, DataField* fieldArray, int numFieldElements, int bufSize, int thresholdPercentage = 80)
      : serialPort(port), fields(fieldArray), numFields(numFieldElements), bufferSize(bufSize) {
      inputBuffer = new char[bufferSize + 1];  // +1 for null termination
      // initialize buffer to zeros
      memset(inputBuffer, 0, (size_t)bufferSize + 1);
      bufferIndex = 0;  // Initialize the buffer index
      // threshold in bytes
      bufferThreshold = max(1, (bufferSize * thresholdPercentage) / 100);
    }

    // Destructor to free the allocated memory
    ~SerialParser() {
      delete[] inputBuffer;  // Free the dynamically allocated buffer
    }

    // Function to read serial data in a non-blocking manner and process input when needed
    void handleSerialInput() {
      while (serialPort.available() > 0) {
        int ch = serialPort.read();  // read returns int
        if (ch < 0) break;           // nothing to read or error

        char incomingChar = (char)ch;

        // Normalize CRLF sequences: treat '\r' like space, '\n' marks end-of-line
        if (incomingChar == '\r') {
          // convert to space to separate tokens
          incomingChar = ' ';
        }

        // If newline is received, process the buffer and reset it
        if (incomingChar == '\n') {
          if (bufferIndex > 0) {
            processBuffer();  // Process the full line of input
            resetBuffer();
          }
          continue;
        }

        // Ensure we have space for the new char + null terminator.
        // If not, process and clear first to avoid overflow.
        if (bufferIndex >= bufferSize - 1) {
          processBuffer();
          resetBuffer();
        }

        // Add the character to the input buffer
        inputBuffer[bufferIndex++] = incomingChar;
        inputBuffer[bufferIndex] = '\0';

        // If we've just received a space, consider parsing partial content to avoid overflow
        if (incomingChar == ' ') {
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
SerialParser parser2(Serial1, fields2, sizeof(fields2) / sizeof(fields2[0]), 256, 80); // Buffer size 256, threshold 80%

void setup() {
  Serial.begin(115200);   // Initialize Serial for the first parser
  // In many Arduino cores Serial1 may not exist; on boards that don't have it, skip init.
#if defined(USART1) || defined(HAVE_HWSERIAL1) || defined(UBRR1H) || defined(Serial1)
  Serial1.begin(115200);  // Initialize Serial1 for the second parser
#endif

  Serial.println("Parser started (parser1 -> Serial). Send lines like: \"input voltage: 12.3 output voltage: 9.8 temperature: 23.5\"");
#if defined(USART1) || defined(HAVE_HWSERIAL1) || defined(UBRR1H) || defined(Serial1)
  Serial1.println("Parser2 on Serial1 started.");
#endif
}

unsigned long lastPrint = 0;
void loop() {
  // Non-blocking serial input handling for both parser instances
  parser1.handleSerialInput();
#if defined(USART1) || defined(HAVE_HWSERIAL1) || defined(UBRR1H) || defined(Serial1)
  parser2.handleSerialInput();
#endif

  // Print parsed values once per second
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();

    Serial.print("Parser1 - Input Voltage: ");
    Serial.println(inputVoltage1, 4);
    Serial.print("Parser1 - Output Voltage: ");
    Serial.println(outputVoltage1, 4);
    Serial.print("Parser1 - Temperature: ");
    Serial.println(temperature1, 4);

#if defined(USART1) || defined(HAVE_HWSERIAL1) || defined(UBRR1H) || defined(Serial1)
    Serial1.print("Parser2 - Input Voltage: ");
    Serial1.println(inputVoltage2, 4);
    Serial1.print("Parser2 - Output Voltage: ");
    Serial1.println(outputVoltage2, 4);
    Serial1.print("Parser2 - Temperature: ");
    Serial1.println(temperature2, 4);
#endif
  }

  // simulate other tasks
  delay(10);
}
