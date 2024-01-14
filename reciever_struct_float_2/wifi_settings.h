//WIFI and UDP multicast includes
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "voltage";
const char* password = "irrolling12";
const char channel = 1; 
const char hidden = 0;
const char max_connection = 8; 
const int beacon_interval = 500; 
const int timeout_period = 1000; // expected delay inbetween packets (for accounting display) 
#define AP_mode_on 1 ; // set to 1 to compile AP node
