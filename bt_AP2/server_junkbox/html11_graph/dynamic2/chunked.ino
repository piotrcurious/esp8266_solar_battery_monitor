#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// Wi-Fi credentials
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Create an instance of the server
AsyncWebServer server(80);

// Define the array of floats
float dataArray[] = {1.0, 2.5, 3.5, 4.0, 5.2, 3.8, 2.2, 1.1};
int dataArraySize = sizeof(dataArray) / sizeof(dataArray[0]);

// Define graph settings
float yMin = 0.0;
float yMax = 6.0;
int barResolution = 10; // Number of '0' characters per unit of float value
int fontSize = 12; // Font size in pixels
int legendFontSize = 16; // Font size for the legend labels

// Handle root URL request
void handleRequest(AsyncWebServerRequest *request) {
  // Begin the response and set headers
  AsyncWebServerResponse *response = request->beginChunkedResponse("text/html", [](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
    String chunk = "";

    if (index == 0) {
      // Start of the HTML
      chunk += "<html><body><table style='width:100%;font-size:" + String(fontSize) + "px;'>";
    }

    static int column = 0; // Track the current column being generated
    float yRange = yMax - yMin;

    if (column < dataArraySize) {
      int barHeight = (dataArray[column] - yMin) / yRange * barResolution;

      // Add the data column
      chunk += "<td style='vertical-align:bottom;text-align:center;'>";
      for (int j = 0; j < barHeight; j++) {
        chunk += "0<br>";
      }
      chunk += "</td>";

      column++;
    } else if (column == dataArraySize) {
      // Add the legend column after all data columns
      chunk += "<td style='vertical-align:middle;text-align:center;'>";

      // Display yMax at the top
      chunk += "<div style='font-size:" + String(legendFontSize) + "px;'>" + String(yMax) + "</div>";

      // Display the last array value in the middle
      float lastValue = dataArray[dataArraySize - 1];
      int lastValuePosition = (lastValue - yMin) / yRange * barResolution;
      for (int j = 0; j < lastValuePosition; j++) {
        chunk += "<br>";
      }
      chunk += "<div style='font-size:" + String(legendFontSize - 2) + "px;'>" + String(lastValue) + "</div>";

      // Display yMin at the bottom
      for (int j = lastValuePosition; j < barResolution; j++) {
        chunk += "<br>";
      }
      chunk += "<div style='font-size:" + String(legendFontSize) + "px;'>" + String(yMin) + "</div>";

      chunk += "</td>";
      column++;
    } else {
      // Close the HTML table and the body
      chunk += "</table></body></html>";
      return chunk.length(); // Return the final chunk and stop sending
    }

    // Copy the chunk to the buffer
    size_t length = chunk.length();
    if (length > maxLen) length = maxLen; // Ensure we don't exceed the buffer size
    memcpy(buffer, chunk.c_str(), length);

    return length; // Return the length of the chunk copied
  });

  // Send the response
  request->send(response);
}

void setup() {
  // Start Serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Start the server and define the root URL handling
  server.on("/", HTTP_GET, handleRequest);

  // Start the server
  server.begin();
}

void loop() {
  // Nothing to do here, everything is handled asynchronously
}
