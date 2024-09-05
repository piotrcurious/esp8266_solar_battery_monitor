// wifi_scanner.h
#ifndef WIFI_SCANNER_H
#define WIFI_SCANNER_H

struct beacon_buf {
  signed rssi : 8;      // RSSI
  uint8_t rxctl[11]; // Unimportant radio info
  uint8_t wifiheader[16];
  uint8_t bssid[6];
  uint8_t fragment[2];
  uint8_t fixed[12];
  uint8_t tagged[76];
  uint16_t cnt;
  uint16_t len;
};

struct ap_info {
  uint8_t bssid[6];
  unsigned char essid[33];
  uint8_t essid_len;
  uint8_t chan;
  signed rssi;
  unsigned long last_seen;
};

void promisc_cb(uint8_t *buf, uint16_t len);
void add_ap(struct ap_info ap);
void print_ap(struct ap_info ap);
void print_all_aps();

#endif // WIFI_SCANNER_H

// wifi_scanner.cpp
#include "wifi_scanner.h"

int known_aps = 0;
struct ap_info aps[MAX_APS];

void promisc_cb(uint8_t *buf, uint16_t len) {
  if (len == 128) {
    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
    struct ap_info ap = parse_tagged(bbuf->tagged);
    
    if(ap.essid_len>0) {
      ap.rssi = bbuf->rssi;
      memcpy(&ap.bssid, bbuf->bssid, 6);
      add_ap(ap);
    }

    if(!memcmp(_settings.wifi_bssid, ap.bssid, 6)) {
      is_beacon_fresh = true; // it's fresh
            
      Serial.print("fresh : ");
      Serial.print(millis());
      Serial.print(" ,delta :");
      Serial.println(millis()-last_beacon_timestamp);             
      last_beacon_timestamp = millis();

    }
  }
}

void add_ap(struct ap_info ap) {
  unsigned long cur_time = millis();
    
  for(int i=0;i<known_aps;i++) {
    if(!memcmp(aps[i].bssid, ap.bssid, 6)) {

      if( ((cur_time - aps[i].last_seen) <= 100 && aps[i].rssi <  ap.rssi) ||
          ((cur_time - aps[i].last_seen) >  100 && aps[i].rssi != ap.rssi) ) {
        
        aps[i].rssi = ap.rssi;
      } 
      aps[i].last_seen = cur_time;
      return;
    }
  }

  memcpy(&aps[known_aps], &ap, sizeof(ap_info));
  aps[known_aps].last_seen = millis();
  if (known_aps < MAX_APS) {
    known_aps++;
  }
}

void print_ap(struct ap_info ap) {
  Serial.printf(" [Ch %02d] [%03d] [%s]\n", ap.chan, ap.rssi, ap.essid );
}

void print_all_aps() {
  int num_shown = 0;
  unsigned long cur_time = millis();

  for(int i=0;i<known_aps;i++) {
    if(cur_time - aps[i].last_seen < 10000) {
      num_shown++;
      print_ap(aps[i]);
    }
    if(num_shown==MAX_APS) return;
  }
}

struct ap_info parse_tagged(uint8_t *tagged) {
  struct ap_info b;
  b.essid_len = 0;
  
  int pos = 0;
  uint8_t len = 0;
  uint8_t tag = 0;
  
  while (pos < 71) {
    tag = tagged[pos];
    len = tagged[pos+1];
    if(len <= 0) return b;
      
    switch(tag) {
      case 0x00:
      
        if(len<32) {
          b.essid_len = len;
          memset(b.essid, 0, 33);
          memcpy(b.essid, &tagged[pos+2], b.essid_len); 
        }
             
        break;
          
      case 0x03:
        b.chan = tagged[pos+2];
        break;
          
      default: break;
    }

    pos += (len + 2);
  }

  return b;
}

// main.cpp
#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "user_interface.h"
#include "SensorFilter.h"
#include "wifi_scanner.h"

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2
#define SLEEP_DURATION 1 * 1000; // duration of sleep . 
//#define SLEEP_DURATION
#define ITERATIONS_PER_WAKEUP 16 // amount of sensor reading iterations per wakeup . 

const bool CONTINOUS_MODE = false ; // option to manually set continous mode

#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T _settings;
#endif

bool initialConnectionEstablished = false;

void yield_delay(uint32_t delay) {
  uint32_t startTime = millis();
  while (millis() - startTime < delay) { // Maximum wait time of 50ms
    yield(); // Allow the ESP8266 to handle background tasks
  }
  // exit after wait
  return ;
}

void fpm_wakup_cb_func(void) {
  Serial.println(F("woke up"));
  Serial.flush();  
  wifi_fpm_close();
}

bool initialConnectAndStoreParams() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  //WiFi.begin(ssid, password);
  WiFi.begin(ssid,password, channel);

  uint32_t timeout = 20*1000; // amount of time segments to wait. different approach because millis change during initial connect even when not yet connected because of beacon synchro
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield_delay(200);
  }
  pinMode(LED_PIN, INPUT);

  if (WiFi.status() == WL_CONNECTED) {
    _settings.ip_address = WiFi.localIP();
    _settings.ip_gateway = WiFi.gatewayIP();
    _settings.ip_mask = WiFi.subnetMask();
    _settings.ip_dns1 = WiFi.dnsIP(0);
    _settings.ip_dns2 = WiFi.dnsIP(1);
    memcpy(_settings.wifi_bssid, WiFi.BSSID(), 6);
    _settings.wifi_channel = WiFi.channel();
    return true;
  }
  return false;
}

void sendUDPPacket(const telemetry_frame& tframe) {
  if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
    size_t bytesWritten = udp.write((uint8_t*)&tframe, sizeof(tframe));
    if (bytesWritten == sizeof(tframe)) {
      udp.endPacket(); // we need to wait anyway.
      yield_delay(20); // just to be sure. if shorter values are used, packets get truncated. it seems there is no other way.
    }
  }
}

void setup_wifi() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT);
  Serial.println("Connecting...");
  uint32_t timing_connecting = millis();
  initialConnectionEstablished = initialConnectAndStoreParams();
  udp.begin(port);
  wifi_fpm_set_wakeup_cb(fpm_wakup_cb_func); // wakeup callback setup

  wifi_promiscuous_enable(0); 
  wifi_set_promiscuous_rx_cb(promisc_cb);   // promiscious packet handler callback setup

  if (initialConnectionEstablished) {
    Serial.println("Connected OK");
    Serial.println(millis()-timing_connecting);
  } else {
    Serial.println("Failed to establish initial connection");
  }
}

void setup() {
  setup_wifi();
}

void reconnectAfterSleep() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    return;
  }

  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(channel);  
  wifi_promiscuous_enable(0); 
  memset(aps,0,sizeof(aps));
  known_aps=0; // reset known apps pointer
  last_beacon_timestamp = millis();

  is_beacon_fresh = false; 
  wifi_promiscuous_enable(1);
   
  uint32_t timeout = millis();
  while(!is_beacon_fresh && millis()-timeout < BEACON_PEEK_TIMEOUT) { yield(); }
  wifi_promiscuous_enable(0);

  if (!is_beacon_fresh) { Serial.println("no AP"); return; } 

  print_all_aps();  

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(_settings.ip_address, _settings.ip_gateway, _settings.ip_mask);
  WiFi.begin(ssid, password, _settings.wifi_channel, _settings.wifi_bssid, true);

  uint32_t timeout = millis();
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && (millis()-timeout) < ASSOCIATE_TIMEOUT_INTERVAL) {
    digitalWrite(LED_PIN, HIGH);
    yield_delay(25);
    digitalWrite(LED_PIN,LOW);
    yield_delay(5);
  }
  pinMode(LED_PIN, INPUT);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to reconnect after sleep");
  }
}

SensorFilter Filter_voltage_ADC0(16, 0.1, 0.05);

void loop() {
  if (digitalRead(MODE_PIN) == HIGH) {
    if (!CONTINOUS_MODE) {
      wifi_fpm_open();
      delay(SLEEP_DURATION);
      wifi_fpm_close();
    }
  }

  for (int i = 0; i < ITERATIONS_PER_WAKEUP; i++) {
    int sensorValue = analogRead(analogInPin);
    float voltage = sensorValue * (5.0 / 1023.0);
    Filter_voltage_ADC0.addSample(voltage);
    yield_delay(10);
  }

  telemetry_frame tframe;
  tframe.voltage = Filter_voltage_ADC0.getFilteredValue();
  sendUDPPacket(tframe);

  if (!CONTINOUS_MODE) {
    wifi_fpm_open();
    delay(SLEEP_DURATION);
    wifi_fpm_close();
    reconnectAfterSleep();
  }
}
