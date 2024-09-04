#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "user_interface.h"
#include "SensorFilter.h" // for filtering of input values using combination of outlier, rolling average and kalman 

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2
#define SLEEP_DURATION 1 * 1000; // duration of sleep . 
//#define SLEEP_DURATION 1 * 100; // duration of sleep . (fast sleep for debug)
#define ITERATIONS_PER_WAKEUP 16 // amount of sensor reading iterations per wakeup . 

//const bool CONTINOUS_MODE = true ; // option to manually set continous mode
const bool CONTINOUS_MODE = false ; // option to manually set continous mode


//#define PERSISTENT_WIFI
//
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
//  WiFi.begin(ssid,password, channel, NULL, true);
  WiFi.begin(ssid,password, channel);

//inital connect synchronizes time , so millis will be changed. 
  uint32_t timeout = 20*1000; // amount of time segments to wait. different approach because millis change during initial connect even when not yet connected because of beacon synchro
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//    yield_delay(200);
    digitalWrite(LED_PIN,LOW); // LOW enables the led
    for (uint16_t how_many_yields_is_ms=0; how_many_yields_is_ms <10; how_many_yields_is_ms++) { // 10x10 
      yield(); //wait for connection. this is because initial wifi connect synchronizes millis
      yield();
      yield();
      yield();
      yield();
      yield();
      yield();
      yield();
      yield();
      yield();
    }
    
    digitalWrite(LED_PIN,HIGH); // HIGH disables the led
    delay(1) ;
    if (timeout > 0) {timeout--;} 
  }    
    pinMode(LED_PIN, INPUT);

// if the connection is not up already
    
WiFi.persistent(true);
WiFi.mode(WIFI_STA);
//WiFi.begin(ssid,password, channel, NULL, true);
  WiFi.begin(ssid,password, channel);

  timeout = millis() ; // 20s timeout
  pinMode(LED_PIN, OUTPUT);
  while (WiFi.status() != WL_CONNECTED && millis() - timeout < 20000) { // millis might change. 
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield_delay(100);
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
            // Wait for the packet to be sent
            yield_delay(20); // just to be sure. if shorter values are used, packets get truncated. it seems there is no other way.
            // No packets pending in the transmit buffer                
    }
  }
}

void setup() {
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


//scanner helper stuff


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
int known_aps = 0;

int rssi_cmp(const void *cmp1, const void *cmp2) {
  struct ap_info ap1 = *((struct ap_info *)cmp1);
  struct ap_info ap2 = *((struct ap_info *)cmp2);

  if(ap2.rssi > ap1.rssi) return 1;
  if(ap1.rssi > ap2.rssi) return -1;
  return 0;
  
}

void print_ap(struct ap_info ap) {
  Serial.printf(" [Ch %02d] [%03d] [%s]\n", ap.chan, ap.rssi, ap.essid );
 // char line[17];
 // snprintf(line, 17, "%16s", ap.essid );
 // oled.putString(line);
}

void add_ap(struct ap_info ap) {

    unsigned long cur_time = millis();
    
    // Check if it is a known ap
    for(int i=0;i<known_aps;i++) {
       if(!memcmp(aps[i].bssid, ap.bssid, 6)) {

          // If it's been less than 100ms, then only update the rssi upward     
          // Otherwise, update it if it has changed.  
          if( ((cur_time - aps[i].last_seen) <= 100 && aps[i].rssi <  ap.rssi) ||
              ((cur_time - aps[i].last_seen) >  100 && aps[i].rssi != ap.rssi) ) {
              
              aps[i].rssi = ap.rssi;
              //qsort(aps, known_aps, sizeof(struct ap_info), rssi_cmp); 
          } 
          aps[i].last_seen = cur_time;
          return;
        }
    }

    memcpy(&aps[known_aps], &ap, sizeof(ap_info));
    aps[known_aps].last_seen = millis();
    if (known_aps < MAX_APS) {
    known_aps++;}

    // Keep it sorted
    //qsort(aps, known_aps, sizeof(struct ap_info), rssi_cmp);
    

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

void promisc_cb(uint8_t *buf, uint16_t len)
{
  if (len == 128) {
    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
    struct ap_info ap = parse_tagged(bbuf->tagged);
    
    if(ap.essid_len>0) {
      ap.rssi = bbuf->rssi;
      memcpy(&ap.bssid, bbuf->bssid, 6);
      add_ap(ap);
    }

// logic to check if "our" ap is on the list
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
// -- end scanner helper stuff


void reconnectAfterSleep() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    return;
  }

  //peek to see if our AP is near
  wifi_set_opmode(STATION_MODE);
  wifi_set_channel(channel);  
  wifi_promiscuous_enable(0); 
//  wifi_set_promiscuous_rx_cb(promisc_cb);
  //aps = {0}; //initalize struct
#ifdef BEACON_DEBUG  
  memset(aps,0,sizeof(aps));
  known_aps=0; // reset known apps pointer
#endif // #ifdef BEACON_DEBUG
  last_beacon_timestamp = millis();

  is_beacon_fresh = false; 
  wifi_promiscuous_enable(1);
   
  uint32_t timeout = millis();
  while(!is_beacon_fresh && millis()-timeout < BEACON_PEEK_TIMEOUT) { yield(); }
  wifi_promiscuous_enable(0);

//  if (millis()-timeout > BEACON_PEEK_TIMEOUT) { Serial.println("no AP"); return; } // if we can't hear the AP , return and do not try to reconnect. 
    if (!is_beacon_fresh) { Serial.println("no AP"); return; } 
#ifdef BEACON_DEBUG
  print_all_aps();  
#endif // #ifdef BEACON_DEBUG

//  wifi_promiscuous_enable(0);
  
// --scanner ends

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(_settings.ip_address, _settings.ip_gateway, _settings.ip_mask);
  WiFi.begin(ssid, password, _settings.wifi_channel, _settings.wifi_bssid, true);

//  uint32_t timeout = millis();
  timeout = millis();

  pinMode(LED_PIN, OUTPUT);
//  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
  while ((WiFi.status() != WL_CONNECTED )&&  ((millis()-timeout) < ASSOCIATE_TIMEOUT_INTERVAL)  ) {

    digitalWrite(LED_PIN, HIGH);
    yield_delay(25);
    digitalWrite(LED_PIN,LOW);
    yield_delay(5);
  }
  pinMode(LED_PIN, INPUT);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to reconnect after sleep");
    //initialConnectionEstablished = false; // no need as settings will stay same
    // it could be useful for a mesh with changing bssid though. 
  }
}



// Create a SensorFilter object 
// 16 entries,
// outlier threshold of 0.1V, 
// base Kalman gain of 0.1, 
// process noise of 0.1, 
// measurement noise of 0.3

SensorFilter Filter_voltage_ADC0(16, 0.1, 0.1, 0.1, 0.3);

// Create a SensorFilter object 
// 16 entries,
// outlier threshold of 2dbm, 
// base Kalman gain of 0.1, (measurement noise) 
// process noise of 0.5, 
// measurement noise of 0.1

SensorFilter Filter_wifi_rssi(128, 2.0, 0.01, 0.8, 0.001);

telemetry_frame tframe; // for entire loop()
float voltage_ADC0 =0 ; // 
float wifi_rssi = 0 ; 

void loop() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    if (!initialConnectionEstablished) {
      yield_delay(1000);  // Wait before retrying
      //todo: sleep in case there is no connection. 
    }
  }

//  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);
//  if (WiFi.status() == WL_CONNECTED) {tframe.wifi_rssi = WiFi.RSSI();}

   for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP; i++) { 
     voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);
     if (WiFi.status() == WL_CONNECTED) {wifi_rssi = WiFi.RSSI();}
     tframe.voltage_ADC0 = Filter_voltage_ADC0.updateSensorReading(voltage_ADC0);
     if (WiFi.status() == WL_CONNECTED) {tframe.wifi_rssi = Filter_wifi_rssi.updateSensorReading(wifi_rssi);}
    }
   
   Serial.println(voltage_ADC0);
   Serial.println(tframe.voltage_ADC0);
     
 // udp.beginPacketMulticast(broadcast, port, WiFi.localIP());
 // udp.write((byte*)&tframe, sizeof(tframe));
 // udp.endPacket();
 // yield_delay(20); // todo : find better way to check if packet is sent

   if (WiFi.status() == WL_CONNECTED) {sendUDPPacket(tframe);} // print how long it took to send a packet in ms. 

  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH && !CONTINOUS_MODE ) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();
    yield_delay(20);
    Serial.flush();

    extern os_timer_t *timer_list;
    timer_list = nullptr;

    wifi_fpm_set_sleep_type(LIGHT_SLEEP_T);
    wifi_fpm_open();

    uint32_t sleepTimeMilliSeconds = SLEEP_DURATION //10 * 1000;
    wifi_fpm_do_sleep(sleepTimeMilliSeconds * 1000);esp_delay(sleepTimeMilliSeconds + 1);
    delay(10);// no idea if nessesary
    yield_delay(20); // no idea if nessesary

    if (sleep_periods_elapsed > SLEEP_PERIODS_TO_SEND_PACKET) {
    sleep_periods_elapsed = 0;  
    if (WiFi.status() != WL_CONNECTED) {reconnectAfterSleep();}
    } else {sleep_periods_elapsed++;}  
    
  } else {
    yield_delay(500); // delay in continuous mode. 3ms minimum. 
    if (WiFi.status() != WL_CONNECTED) {reconnectAfterSleep();}
  }
}
