#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
// to disable wifi sleep mode

// Declare a global variable to store the asyncUDP object
static AsyncUDP udp;

const char* ssid = "voltage2";
const char* password = "irrolling12";
const uint8_t channel = 1; 
const bool hidden = 0;
const uint8_t max_connection = 8; // 4 connections for less memory. 8 max.  
const uint16_t beacon_interval = 100; 

const uint16_t multicastPort = 5683;  // local port to listen on
static const IPAddress multicastIP(224,0,1,187);