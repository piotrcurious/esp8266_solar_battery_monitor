
// simple POC : send hello world string over UDP-multicast address every 2 seconds.
// a very simple way to connect IOT to local network without need of any server.
// author : Marc Quinton / feb 2019
// keywords : esp8266, arduino, multicast, IOT, example, POC

#include "wifi_settings.h"
#include "telemetry_frame.h"

WiFiUDP udp;
const int port = 5683;

// IPAddress broadcast;
IPAddress broadcast=IPAddress(224, 0, 1, 187);


// ADC pin
const int analogInPin = A0;

//LED pin

#define LED_PIN 2 // use GPIO2 (bootselector/LED pin)
#define MODE_PIN 4 // use GPIO4 (i2c pin)


void setup(void)
{
  Serial.begin(115200);
//  Serial.println("Startup");
  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch
  WiFi.mode(WIFI_STA);
 // WiFi.softAP(ssid, password, channel, hidden, max_connection);
  WiFi.begin(ssid, password);

    /**
    * datasheet:
    *
   wifi_set_sleep_level():
   Set sleep level of modem sleep and light sleep
   This configuration should be called before calling wifi_set_sleep_type
   Modem-sleep and light sleep mode have minimum and maximum sleep levels.
   - In minimum sleep level, station wakes up at every DTIM to receive
     beacon.  Broadcast data will not be lost because it is transmitted after
     DTIM.  However, it can not save much more power if DTIM period is short,
     as specified in AP.
   - In maximum sleep level, station wakes up at every listen interval to
     receive beacon.  Broadcast data may be lost because station may be in sleep
     state at DTIM time.  If listen interval is longer, more power will be saved, but
     it’s very likely to lose more broadcast data.
   - Default setting is minimum sleep level.
   Further reading: https://routerguide.net/dtim-interval-period-best-setting/

   wifi_set_listen_interval():
   Set listen interval of maximum sleep level for modem sleep and light sleep
   It only works when sleep level is set as MAX_SLEEP_T
   forum: https://github.com/espressif/ESP8266_NONOS_SDK/issues/165#issuecomment-416121920
   default value seems to be 3 (as recommended by https://routerguide.net/dtim-interval-period-best-setting/)

   call order:
     wifi_set_sleep_level(MAX_SLEEP_T) (SDK3)
     wifi_set_listen_interval          (SDK3)
     wifi_set_sleep_type               (all SDKs)

    */

  
//  WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 10);
  WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);

  //WiFi.setPhyMode(WIFI_PHY_MODE_11B); // max range 
//  Serial.println("");

//  Serial.print("broadcast address : ");
//  Serial.print(broadcast);
//  Serial.print(":");
//  Serial.println(port);

// blink led until connected
  
    pinMode(LED_PIN, OUTPUT);
    while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN,LOW);
    delay(500);
    digitalWrite(LED_PIN,HIGH);
    delay(500);
    }
    pinMode(LED_PIN, INPUT);
    pinMode(MODE_PIN, INPUT);

// deeper sleep pin - high makes device update less often and
// sleep in meantime
// low updates with 20ms intervals. 

  udp.begin(port);
}

int UDP_packets = 0 ; 
void loop(void)
{
struct telemetry_frame tframe ;
  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);

  udp.beginPacketMulticast(broadcast, port, WiFi.localIP());
  udp.write((byte*)&tframe,sizeof(tframe));
  udp.endPacket();

  if (digitalRead(MODE_PIN) == HIGH) {  //only visualise RSSI if no high speed mode is used 
                                        // as it introduces useless delay otherwise 
  int8_t rssi = WiFi.RSSI();
  uint16_t intensity_blue = map (rssi, -60,-95, 255,0);
  intensity_blue = constrain(intensity_blue, 0, 255);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN,LOW);
    delay(intensity_blue); // signal 
    digitalWrite(LED_PIN,HIGH);
    pinMode(MODE_PIN, INPUT);
//    delay(20);
  }  
  UDP_packets++;

  if (digitalRead(MODE_PIN) == HIGH){
   //WiFi.forceSleepBegin();  // Wifi Off
   delay(1000*10); // sleep for 10 seconds
   //WiFi.forceSleepWake();  // Wifi On
   // while (WiFi.status() != WL_CONNECTED) {
   // pinMode(D4, OUTPUT);
   // digitalWrite(D4,LOW);
   // delay(500);
   // digitalWrite(D4,HIGH);
   // delay(500);
   // pinMode(D4, INPUT);
  //  FIXME -- it all does not work, after few reconnects
  //  with weak signal, it ends up disconnected. 
  //  reconnects eat up more energy than expected too 
  //  }
  }
  else {
    delay (20); // no sleep mode, send packets as quick as reasonably possible. 
  }
  
}
