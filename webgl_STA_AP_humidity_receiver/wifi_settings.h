#ifndef WIFI_SETTINGS_H
#define WIFI_SETTINGS_H

#include <WiFi.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include <esp_wifi.h>

// AP settings
static const char* ap_ssid = "humidity_ap";
static const char* ap_password = "password123";

// STA settings (to connect to local network)
static const char* sta_ssid = "your_local_wifi";
static const char* sta_password = "your_local_password";

static const uint16_t multicastPort = 5683;
static const IPAddress multicastIP(224,0,1,187);

#endif
