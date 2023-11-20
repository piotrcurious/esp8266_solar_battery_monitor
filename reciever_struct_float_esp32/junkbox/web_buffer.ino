#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>

// Define the AP credentials
const char* ssid = "ESP32_AP";
const char* password = "12345678";

// Define the UDP parameters
const IPAddress multicastIP(224, 0, 1, 187);
const int udpPort = 5683;

// Define the buffer size and packet size
const int bufferSize = 100;
const int packetSize = 1500;

// Define the web server port
const int webPort = 80;

// Create a UDP object
WiFiUDP udp;

// Create a web server object
WebServer server(webPort);

// Create a buffer to store the packets
byte buffer[bufferSize][packetSize];

// Create a variable to keep track of the buffer index
int bufferIndex = 0;

// A function to handle the web server requests
void handleRoot() {
  // Check if the request is for buffer.bin
  if (server.uri() == "/buffer.bin") {
    // Send the buffer as binary data
    server.send(200, "application/octet-stream", (const char*)buffer, bufferSize * packetSize);
  } else {
    // Send a simple message
    server.send(200, "text/plain", "Hello from ESP32!");
  }
}

// A function to handle the web server requests
void handleClient () {
  // Check if the requested file is the buffer file
  if (server.uri() == fileName) {
    // Set the content type to application/octet-stream
    server.setContentLength(bufferSize * packetSize);
    server.send(200, "application/octet-stream", "");
    // Send the buffer data in chunks
    for (int i = 0; i < bufferSize; i++) {
      server.client().write(buffer[i], packetSize);
    }
  } else {
    // Send a 404 response for other files
    server.send(404, "text/plain", "File not found");
  }
}
// A function to handle the UDP packets
void handleUDP() {
  // Check if there is any packet available
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Read the packet into the buffer
    udp.read(buffer[bufferIndex], packetSize);

    // Increment the buffer index
    bufferIndex++;

    // Check if the buffer is full
    if (bufferIndex == bufferSize) {
      // Shift the buffer elements to the left by one
      for (int i = 0; i < bufferSize - 1; i++) {
        memcpy(buffer[i], buffer[i + 1], packetSize);
      }

      // Reset the buffer index to the last position
      bufferIndex = bufferSize - 1;
    }
  }
}

void setup() {
  // Initialize the serial monitor
  Serial.begin(115200);

  // Initialize the WiFi in AP mode
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  // Print the AP IP address
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Begin the UDP connection
  udp.begin(udpPort);

  // Join the multicast group
  udp.joinMulticast(multicastIP);

  // Print the UDP parameters
  Serial.print("UDP multicast IP: ");
  Serial.println(multicastIP);
  Serial.print("UDP port: ");
  Serial.println(udpPort);

  // Start the web server
  server.on("/", handleRoot);
  server.begin();

  // Print the web server port
  Serial.print("Web server port: ");
  Serial.println(webPort);
}

void loop() {
  // Handle the UDP packets
  handleUDP();

  // Handle the web server requests
  server.handleClient();
}


