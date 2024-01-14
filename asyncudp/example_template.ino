#include <ESPAsyncUDP.h>

// create a struct to store the incoming packet data
struct PacketData {
  byte buffer[500]; // 500 byte buffer
  IPAddress ip; // source IP address
  uint16_t port; // source port
};

// create an AsyncUDP object
AsyncUDP udp;

// create a callback function to handle incoming multicast packets
void onPacket(AsyncUDPPacket packet) {
  // create a PacketData object
  PacketData data;
  // copy the packet data to the buffer
  memcpy(data.buffer, packet.data(), packet.length());
  // copy the source IP and port
  data.ip = packet.remoteIP();
  data.port = packet.remotePort();
  // do something with the data
  // for example, print it to the serial monitor
  Serial.print("Received multicast packet from ");
  Serial.print(data.ip.toString());
  Serial.print(":");
  Serial.println(data.port);
  Serial.print("Data: ");
  for (int i = 0; i < packet.length(); i++) {
    Serial.print((char)data.buffer[i]);
  }
  Serial.println();
}

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  // connect to WiFi network
  WiFi.begin("your-ssid", "your-password");
  // wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  // print the local IP address
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // start listening on the multicast address and port
  if (udp.listenMulticast(IPAddress(192, 168, 1, 244), 5234)) {
    Serial.println("UDP multicast listening");
    // set the callback function for incoming packets
    udp.onPacket(onPacket);
  }
  else {
    Serial.println("UDP multicast failed");
  }
}

void loop() {
  // nothing to do here
}
