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

// Function to generate ASCII art bars with legend
String generateAsciiGraph() {
  String html = "<html><body><table style='width:100%;font-size:" + String(fontSize) + "px;'>";
  
  // Determine the dynamic height scale
  float yRange = yMax - yMin;

  // Create the graph bars
  for (int i = 0; i < dataArraySize; i++) {
    // Map the float value to the number of '0' characters
    int barHeight = (dataArray[i] - yMin) / yRange * barResolution;

    // Generate the table column with the ASCII art bar
    html += "<td style='vertical-align:bottom;text-align:center;'>";
    for (int j = 0; j < barHeight; j++) {
      html += "0<br>";
    }
    html += "</td>";
  }

  // Add the legend column
  html += "<td style='vertical-align:middle;text-align:center;'>";
  
  // Display yMax at the top
  html += "<div style='font-size:" + String(legendFontSize) + "px;'>" + String(yMax) + "</div>";

  // Display the last array value in the middle
  float lastValue = dataArray[dataArraySize - 1];
  int lastValuePosition = (lastValue - yMin) / yRange * barResolution;
  for (int j = 0; j < lastValuePosition; j++) {
    html += "<br>";
  }
  html += "<div style='font-size:" + String(legendFontSize - 2) + "px;'>" + String(lastValue) + "</div>";
  
  // Display yMin at the bottom
  for (int j = lastValuePosition; j < barResolution; j++) {
    html += "<br>";
  }
  html += "<div style='font-size:" + String(legendFontSize) + "px;'>" + String(yMin) + "</div>";

  html += "</td>";
  
  html += "</table></body></html>";
  return html;
}

// Handle root URL request
void handleRequest(AsyncWebServerRequest *request) {
  String html = generateAsciiGraph();
  request->send(200, "text/html", html);
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
