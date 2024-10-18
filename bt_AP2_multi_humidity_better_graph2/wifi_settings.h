#define AP_SSID "voltage"
#define BT_SSID "voltage"

#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_pm.h>
#include <esp_wifi.h>
#include <esp_wifi_types.h>
// to disable wifi sleep mode

//bt

#include <BluetoothSerial.h>
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_main.h"
#include "esp_coexist.h"

#include "esp_intr_alloc.h"

// Declare a global variable to store the asyncUDP object
static AsyncUDP udp;

static const char* ssid = AP_SSID;
static const char* password = "irrolling12";
static const uint8_t channel = 1; 
static const bool hidden = 0;
static const uint8_t max_connection = 8; // 4 connections for less memory. 8 max.  
static const uint16_t beacon_interval = 500; 

static const uint16_t multicastPort = 5683;  // local port to listen on
static const IPAddress multicastIP(224,0,1,187);
