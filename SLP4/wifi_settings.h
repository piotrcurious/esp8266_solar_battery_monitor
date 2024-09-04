//esp32
//#include <WiFi.h>
//#include <AsyncUDP.h>
//#include <esp_pm.h>
//#include <esp_wifi.h>
//#include <esp_wifi_types.h>
// to disable wifi sleep mode

//WIFI and UDP multicast includes
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define ASSOCIATE_TIMEOUT_INTERVAL 10*1000 // 10seconds
#define SLEEP_PERIODS_TO_SEND_PACKET 10 // send packet only once per [x] sleep periods. other wakeups only update the filter.
uint8_t sleep_periods_elapsed = 0 ; // global variable to keep track of amount of sleep periods


//esp32
// Declare a global variable to store the asyncUDP object
//AsyncUDP udp;
//uint16_t multicastPort = 5683;  // local port to listen on
//IPAddress multicastIP(224,0,1,187);

WiFiUDP udp;
const int port = 5683;

// IPAddress broadcast;
IPAddress broadcast=IPAddress(224, 0, 1, 187);

const char* ssid = "voltage2";
const char* password = "irrolling12";
const uint8_t channel = 1; 
const bool hidden = 0;
const uint8_t max_connection = 8; 
const uint16_t beacon_interval = 100; 

bool is_beacon_fresh = false ; // set by promiscious sniffer
uint32_t last_beacon_timestamp = 0; // 
#define BEACON_PEEK_TIMEOUT beacon_interval*10 // do not attempt to reconnect if no beacon after this period
#define BEACON_DEBUG 

#define PERSISTENT_WIFI  // attempt to use tricks to quickly reconnect to wifi 

#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T {
//  struct _settings {
  uint16_t magic;
  uint32_t ip_address;
  uint32_t ip_gateway;
  uint32_t ip_mask;
  uint32_t ip_dns1;
  uint32_t ip_dns2;
  char wifi_ssid[50];
  char wifi_auth[50];
  uint8_t wifi_bssid[6];
  uint16_t wifi_channel;
}; // size = 132 bytes
#endif //PERSISTENT_WIFI
