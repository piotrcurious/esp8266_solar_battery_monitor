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

//#define BEACON_DEBUG 
#define CURRENT_BEACON_DEBUG // report fact of capturing AP nearby - comment out before deployment 
#define PACKET_SENT_TIME_DEBUG // report how long it took to reassociate and send packet

#define ASSOCIATE_TIMEOUT_INTERVAL 10*1000 // 10seconds
//#define ASSOCIATE_TIMEOUT_INTERVAL 500 // 500ms // if power saving is important, shorten the association time - packets which are not accepted on time will be lost

#define SLEEP_PERIODS_TO_SEND_PACKET 10 // send packet only once per [x] sleep periods. other wakeups only update the filter.
uint8_t sleep_periods_elapsed = 0 ; // global variable to keep track of amount of sleep periods
bool transmit_phase = false ; // global flag to indicate if we are in transmit phase and we should wait for connection being established. 


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


// structures usefull for wifi scanner
// todo: cleanup - not everything is needed just to detect BSSID


struct beacon_buf {

  // Radio Information
  signed rssi : 8;      // RSSI
  uint8_t rxctl[11]; // Unimportant radio info

  // 112 Bytes of Frame
  
  // - Header
  uint8_t wifiheader[16];
  uint8_t bssid[6];
  uint8_t fragment[2];
  
  // - Fixed 802.11 Parameters
  uint8_t fixed[12];

  // - 100 bytes of tagged 802.11 Parameters
  // - The SSID, which we are interested in, should be here
  uint8_t tagged[76];

  // Other WiFi Information
  uint16_t cnt;      
  uint16_t len;
  
};

#ifdef BEACON_DEBUG
int known_aps = 0;

struct ap_info {
  uint8_t bssid[6];
  unsigned char essid[33];
  uint8_t essid_len;
  uint8_t chan;
  signed rssi;
  unsigned long last_seen;
};

// Room for 15 APs
#define MAX_APS 15 // max AP's in the list
struct ap_info aps[MAX_APS];
#endif // #ifdef BEACON_DEBUG
