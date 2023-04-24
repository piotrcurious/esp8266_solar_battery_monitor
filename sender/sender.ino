#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
//#include <ESP8266WebServer.h>

// simple POC : send hello world string over UDP-multicast address every 2 seconds.
// a very simple way to connect IOT to local network without need of any server.
// author : Marc Quinton / feb 2019
// keywords : esp8266, arduino, multicast, IOT, example, POC

WiFiUDP udp;
const int port = 5683;

// IPAddress broadcast;
IPAddress broadcast=IPAddress(224, 0, 1, 187);

const char* ssid = "sensor1";
const char* password = "testudp1234";
const char channel = 1; 
const char hidden = 0;
const char max_connection = 4; 

void setup(void)
{
  Serial.begin(115200);
//  Serial.println("Startup");

  WiFi.mode(WIFI_STA);
 // WiFi.softAP(ssid, password, channel, hidden, max_connection);
  WiFi.begin(ssid, password);
//  Serial.println("");

//  Serial.print("broadcast address : ");
//  Serial.print(broadcast);
//  Serial.print(":");
//  Serial.println(port);
 udp.begin(port);
}

void sendUDP(String string) {

//  Serial.print("sendUDP : ");
//  Serial.println(string);

  // convert string to char array
  char msg[255];
  string.toCharArray(msg,255);

  udp.beginPacketMulticast(broadcast, port, WiFi.localIP());
  udp.write(msg);
  udp.endPacket();
}

int UDP_packets = 0 ; 
void loop(void)
{
//  sendUDP((String)"hello world");
  sendUDP((String)UDP_packets);
  UDP_packets++;
  delay(50);
}
