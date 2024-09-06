#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include "user_interface.h"
#include "SensorFilter.h" // for filtering of input values using combination of outlier, rolling average and kalman 

#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2
#define SLEEP_DURATION 1 * 1000; // duration of sleep . 
//#define SLEEP_DURATION 1 * 100; // duration of sleep . (fast sleep for debug)
#define ITERATIONS_PER_WAKEUP 16 // amount of sensor reading iterations per wakeup . 
#define ITERATIONS_PER_WAKEUP_RSSI 32 // amount of sensor reading iterations per wakeup - RSSI. 
#define ITERATIONS_PER_WAKEUP_SHT4x 8 // amount of sensor reading iterations per wakeup - RSSI. 

const bool idle_sensor_update = true ; // true means sensors get updated at each wakeup. 
                            //false means that sensors are updated only when it's time to send packet and AP is detected

//const bool CONTINOUS_MODE = true ; // option to manually set continous mode (sending without sleep)
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
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks

          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
          yield(); // Allow the ESP8266 to handle background tasks
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
  pinMode(LED_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);

if (! sht4.begin()) {
    Serial.println("Couldn't find SHT4x");
  }


// You can have 3 different precisions, higher precision takes longer
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  switch (sht4.getPrecision()) {
     case SHT4X_HIGH_PRECISION: 
       Serial.println("High precision");
       break;
     case SHT4X_MED_PRECISION: 
       Serial.println("Med precision");
       break;
     case SHT4X_LOW_PRECISION: 
       Serial.println("Low precision");
       break;
  }


// You can have 6 different heater settings
  // higher heat and longer times uses more power
  // and reads will take longer too!
  sht4.setHeater(SHT4X_HIGH_HEATER_1S);  // setting to high heat to decontaminate upon startup
  switch (sht4.getHeater()) {
     case SHT4X_NO_HEATER: 
       Serial.println("No heater");
       break;
     case SHT4X_HIGH_HEATER_1S: 
       Serial.println("High heat for 1 second");
       break;
     case SHT4X_HIGH_HEATER_100MS: 
       Serial.println("High heat for 0.1 second");
       break;
     case SHT4X_MED_HEATER_1S: 
       Serial.println("Medium heat for 1 second");
       break;
     case SHT4X_MED_HEATER_100MS: 
       Serial.println("Medium heat for 0.1 second");
       break;
     case SHT4X_LOW_HEATER_1S: 
       Serial.println("Low heat for 1 second");
       break;
     case SHT4X_LOW_HEATER_100MS: 
       Serial.println("Low heat for 0.1 second");
       break;
  }
  
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

  sht4.setHeater(SHT4X_NO_HEATER);  // disabling heater

}


//scanner helper stuff

#ifdef BEACON_DEBUG
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
#endif // #ifdef BEACON_DEBUG

void promisc_cb(uint8_t *buf, uint16_t len)
{
    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
#ifdef BEACON_DEBUG
  if (len == 128) {
//    struct beacon_buf *bbuf = (struct beacon_buf*) buf;
    struct ap_info ap = parse_tagged(bbuf->tagged);

    
    if(ap.essid_len>0) {
      ap.rssi = bbuf->rssi;
      memcpy(&ap.bssid, bbuf->bssid, 6);
      add_ap(ap);
    }
#endif //#ifdef BEACON_DEBUG

// logic to check if "our" ap is on the list
//    if(!memcmp(_settings.wifi_bssid, ap.bssid, 6)) {
   if(!memcmp(_settings.wifi_bssid, bbuf->bssid, 6)) {
      is_beacon_fresh = true; // it's fresh

#ifdef CURRENT_BEACON_DEBUG            
      Serial.print("fresh : ");
      Serial.print(millis());
      Serial.print(" ,delta :");
      Serial.println(millis()-last_beacon_timestamp);             
#endif //#ifdef CURRENT_BEACON_DEBUG
      last_beacon_timestamp = millis(); // TODO: not used, but could be used to defer reconnect to not waste energy waiting for AP beacon
    } 
#ifdef BEACON_DEBUG
  }// if (len == 128) {
#endif // #ifdef BEACON_DEBUG  
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

  transmit_phase = true; // we initiated connection. indicate it to routines expecting wifi to be up. 
} // then we exit as there is no need to block 


void wait_for_connection () {
//  uint32_t timeout = millis();
  uint32_t timeout = millis();
  pinMode(LED_PIN, OUTPUT);
//  while (WiFi.status() != WL_CONNECTED && millis() < timeout) {
  while ((WiFi.status() != WL_CONNECTED )&&  ((millis()-timeout) < ASSOCIATE_TIMEOUT_INTERVAL)  ) {
    digitalWrite(LED_PIN, HIGH);
    yield_delay(25);
    digitalWrite(LED_PIN,LOW);
    yield_delay(5);
  }
  pinMode(LED_PIN, INPUT);

//  if (WiFi.status() != WL_CONNECTED) {
//    Serial.println("Failed to reconnect after sleep");
    //initialConnectionEstablished = false; // no need as settings will stay same
    // it could be useful for a mesh with changing bssid though. 
//  }

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

// Create a SensorFilter object 
// 16 entries,
// outlier threshold of 0.1C, 
// base Kalman gain of 0.1, 
// process noise of 0.1, 
// measurement noise of 0.3

SensorFilter Filter_aTemperature(32, 0.1, 0.01, 0.01, 0.001);

// Create a SensorFilter object 
// 16 entries,
// outlier threshold of 0.2rh, 
// base Kalman gain of 0.1, 
// process noise of 0.1, 
// measurement noise of 0.3

SensorFilter Filter_aHumidity(32, 0.2, 0.01, 0.01, 0.001);

telemetry_frame tframe; // for entire loop()
float voltage_ADC0 =0 ; // external voltage
uint32_t radio_active_time = 0 ;// time spent for re-association and sending packet , with radio module active
float wifi_rssi = 0 ; 
// SHT3x globals
float aTemperature = 0.0;
float aHumidity = 0.0;

void loop() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    if (!initialConnectionEstablished) {
      yield_delay(1000);  // Wait before retrying
      //todo: sleep in case there is no connection. 
    }
  }

#ifdef PACKET_SENT_TIME_DEBUG
uint32_t wait_for_connection_timer ; 
if (transmit_phase) { wait_for_connection_timer = millis();}
#endif //#ifdef PACKET_SENT_TIME_DEBUG

//  sensors read on each wakeup go here :
//  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);
//  if (WiFi.status() == WL_CONNECTED) {tframe.wifi_rssi = WiFi.RSSI();}
     

  if (transmit_phase || idle_sensor_update) {
   for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP; i++) { 
     voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);
     tframe.voltage_ADC0 = Filter_voltage_ADC0.updateSensorReading(voltage_ADC0);
    } // update sensors which do not need wifi 


  sensors_event_t humidity, temp;
/*  
  uint32_t timestamp = millis();
  sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
  timestamp = millis() - timestamp;

  Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" degrees C");
  Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println("% rH");

  Serial.print("Read duration (ms): ");
  Serial.println(timestamp);
*/

   for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP_SHT4x; i++) { 
     sht4.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data
     tframe.SH4x_rel_humidity = Filter_aHumidity.updateSensorReading(humidity.relative_humidity) ; 
     tframe.SH4x_temperature = Filter_aTemperature.updateSensorReading(temp.temperature);  
    } // update sensors which do not need wifi 

//    Serial.print("Temperature: "); Serial.print(tframe.SH4x_temperature);// Serial.println(" degrees C");
//    Serial.print("Humidity: "); Serial.print(tframe.SH4x_rel_humidity); //Serial.println("% rH");
    
  if (transmit_phase) {
      wait_for_connection(); //wait for connection and other things only if it's transmit phase 
      for (uint8_t i = 0 ; i < ITERATIONS_PER_WAKEUP_RSSI; i++) { 
         if (WiFi.status() == WL_CONNECTED) {wifi_rssi = WiFi.RSSI();}
         if (WiFi.status() == WL_CONNECTED) {tframe.wifi_rssi = Filter_wifi_rssi.updateSensorReading(wifi_rssi);}
        } // update things which need wifi
      }
  }
//   Serial.println(voltage_ADC0);
//   Serial.println(tframe.voltage_ADC0);
//     Serial.println(tframe.voltage_VCC33);

     if (transmit_phase) {
      radio_active_time = millis()-wait_for_connection_timer;
      tframe.radio_active_time = radio_active_time ;
     }

   if (WiFi.status() == WL_CONNECTED) {sendUDPPacket(tframe);} // send the packet whenever we are connected
//radio_active_time = millis()-wait_for_connection_timer;
#ifdef PACKET_SENT_TIME_DEBUG
if (transmit_phase) { Serial.print("sent in : "); Serial.println(radio_active_time); }
#endif //#ifdef PACKET_SENT_TIME_DEBUG

  pinMode(MODE_PIN, INPUT);
  if (digitalRead(MODE_PIN) == HIGH && !CONTINOUS_MODE ) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_OFF);
    transmit_phase = false ; // indicate that we are offline
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
    transmit_phase = true ; //always on
    yield_delay(500); // delay in continuous mode. 3ms minimum. 
    if (WiFi.status() != WL_CONNECTED) {transmit_phase = false ; reconnectAfterSleep();}
  }
}
