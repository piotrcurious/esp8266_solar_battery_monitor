#include "wifi_settings.h" // see to enable AP mode too
#include "telemetry_frame.h" // make sure to sync that with sender

telemetry_frame tframe; // create global struct telemetry frame. 

// create an AsyncUDP object
AsyncUDP udp;

// create a callback function to handle incoming multicast packets
void onPacket(AsyncUDPPacket packet) {

  // copy the packet data to the buffer
  memcpy((byte*)&tframe, packet.data(), sizeof(tframe));
  // copy the source IP and port
//  data.ip = packet.remoteIP();
//  data.port = packet.remotePort();
  // do something with the data
  // for example, print it to the serial monitor
//  Serial.print("Received multicast packet from ");
//  Serial.print(data.ip.toString());
//  Serial.print(":");
//  Serial.println(data.port);
  Serial.print("Voltage: ");
  Serial.print(tframe.voltage_ADC0,2);
  Serial.println();
}

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  // connect to WiFi network
  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch

#ifdef AP_mode_on // if ap mode , start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.print("WiFi: ");
//  display.print(millis());
//  display.display();
    delay(1000);
//    Serial.println("Connecting to WiFi...");

    }
//  display.clearDisplay();
//  display.display();   
#endif AP_mode_on 
  
  // print the local IP address
//  Serial.println("WiFi connected");
//  Serial.print("IP address: ");
//  Serial.println(WiFi.localIP());

  // start listening on the multicast address and port
//  if (udp.listenMulticast(multicastIP, multicastPort)) {
//    Serial.println("UDP multicast listening");
    // set the callback function for incoming packets
//    udp.onPacket(onPacket);
//  }
//  else {
//    Serial.println("UDP multicast failed");
//  }

  // start listening on the multicast address and port
  udp.listenMulticast(multicastIP, multicastPort);
    // set the callback function for incoming packets
  udp.onPacket(onPacket);

}

void loop() {
  // nothing to do here
  delay(1);
}
