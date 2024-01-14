/*
 * SSD1306 multicast reciever
 * it gets data from multicast frames sent by AP 
 * and displays it on SSD1306 display
 * it should :
 * - get wifi SSID and password thru IR (infrared) port
 * - maintain frames synchronized in time 
 * - have interrupt framework for handling frame assembly and display
 * - have display modes and feedback via encoder, buttons etc. 
 */

//wifi config
#include "wifi_settings.h" // see to enable AP mode too
#include "telemetry_frame.h" // make sure to sync that with sender

WiFiUDP Udp;
unsigned int multicastPort = 5683;  // local port to listen on
IPAddress multicastIP(224,0,1,187);

#include "display_settings.h" // includes configuration of on-screen widgets

//timer interrupt includes
// Select a Timer Clock
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

#include "ESP8266TimerInterrupt.h"
#include "ESP8266_ISR_Timer.h"

// end of timer interrupt includes

// --- timer interrupt init 
#define HW_TIMER_INTERVAL_MS      10L
// Init ESP8266 timer 1
ESP8266Timer ITimer;
// Init ISR timer 
ESP8266_ISR_Timer ISR_Timer;
// --- end timer interrupt init

//timer interrupt helper functions
void IRAM_ATTR TimerHandler()
{
  ISR_Timer.run();
}
#define NUMBER_ISR_TIMERS         2

// You can assign any interval for any timer here, in milliseconds
uint32_t TimerInterval[NUMBER_ISR_TIMERS] =
{
  20L,   // OLED refresh, display interface interrupt 
  10L  // UDP/network layer interface interrupt
};

typedef void (*irqCallback)  ();
//end of timer interrupt helper functions

//timer interrupt functions 
bool display_refresh_sync;
void display_interface_interrupt(){
 display_refresh_sync=0; //clear display_refresh_sync at regular intervals
}

void UDP_interface_interrupt(){

}

//set upt the irq callback array
irqCallback irqCallbackFunc[NUMBER_ISR_TIMERS] =
{
  display_interface_interrupt,  
  UDP_interface_interrupt
};
//end of timer interrupt functions

uint last_packet_millis = 0 ; 
uint wifi_bars_millis = 0 ; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
//  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
//    Serial.println(F("SSD1306 allocation failed"));
//  //  for(;;); // Don't proceed, loop forever
//  }

delay(250);
display.begin(SCREEN_ADDRESS,true);
display.display();
delay(2000);


  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
    // Clear the buffer
  display.clearDisplay();
  // Draw a single pixel in white aka "Ready"
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(1000);

  // start timer interrupts
    ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler) ;
      for (uint16_t i = 0; i < NUMBER_ISR_TIMERS; i++)
  {
    ISR_Timer.setInterval(TimerInterval[i], irqCallbackFunc[i]);
  }

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw white text on a black background
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
    
  // WiFi.begin(ssid, password);
  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch

#ifdef AP_mode_on // if ap mode , start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection, beacon_interval);
#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
  display.setCursor(0, 0);     // Start at top-left corner
  display.print("WiFi: ");
  display.print(millis());
  display.display();
    }
  display.clearDisplay();
  display.display();   
#endif AP_mode_on 

  Udp.beginMulticast(WiFi.localIP(), multicastIP, multicastPort); // start multicast 

  last_packet_millis = millis() ;
  wifi_bars_millis   = millis() ;
}

uint UDP_packets = 0;
uint UDP_packets_lost = 0 ; 
uint UDP_packets_lost_total = 0 ; 

void display_wifi_bars() {
  volatile int RSSI=WiFi.RSSI();
volatile uint8_t bars = 0;
if (RSSI > -55) { 
    bars = 4;
  } else if (RSSI < -55 & RSSI > -65) {
    bars = 4;
  } else if (RSSI < -65 & RSSI > -75) {
    bars = 3;
  } else if (RSSI < -75 & RSSI > -85) {
    bars = 2;
  } else if (RSSI < -85 & RSSI > -95) {
    bars = 1;
  } else {
    bars = 0;
  }

  display.fillRect (WIFI_BARS_X,WIFI_BARS_Y-8,10,8,SSD1306_BLACK);
  for (uint8_t b=0; b <= bars; b++) {
    display.fillRect(WIFI_BARS_X + (b*2),WIFI_BARS_Y -(b*2),1,b*2,SSD1306_WHITE); 
  }
  if (!bars) {display.drawPixel(WIFI_BARS_X,WIFI_BARS_Y-1 ,SSD1306_WHITE);}
  
}
 
void loop() {
  struct telemetry_frame tframe ; // use telemetry_frame struct from telemetry_frame.h
if (!display_refresh_sync) {
//  display.setTextSize(1);      // Normal 1:1 pixel scale
//  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw white text on a black background
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.cp437(true);         // Use full 256 char 'Code Page 437' font
//  display.print(millis());
   //   display.setCursor(10,3*8-1);
   //   display.setTextSize(1);
    //  display.print(WiFi.RSSI());
  display.display();
  display_refresh_sync=1; //set display_refresh_sync flag so we do not refresh unnessesairly
}
    int packetSize = Udp.parsePacket(); 
       if ((millis() - wifi_bars_millis) > WIFI_BARS_PERIOD) {
        display_wifi_bars();
        wifi_bars_millis=millis();
       }
    if (packetSize){
// packet accounting
    UDP_packets++;
    last_packet_millis = millis(); // reset last packet counter
        if (UDP_packets_lost>0){
          if (UDP_packets_lost>=TIMEOUT_TICKER_WIDTH){ UDP_packets_lost = TIMEOUT_TICKER_WIDTH;}
          for (uint8 i = 0; i <= UDP_packets_lost; i++) {
            display.drawPixel(TIMEOUT_TICKER_X+i,TIMEOUT_TICKER_Y,SSD1306_BLACK);
          }
        }
    UDP_packets_lost = UDP_packets_lost/2; // divide lost packets counter by two to create persistent average 
    if (UDP_packets_lost>0){
      display.drawPixel(TIMEOUT_TICKER_X+UDP_packets_lost,TIMEOUT_TICKER_Y,SSD1306_WHITE);
    }
    int message_size=Udp.read((byte*)&tframe,packetSize); // read the packet into the buffer

    if (message_size>0){
//      packetBuffer[message_size]='\0'; //// null terminate

      display.setCursor(0, 0);
      display.setTextSize(2);      // Normal 1:1 pixel scale
      display.print(tframe.voltage_ADC0,3);
      display.print(F("V "));

//      display.setCursor(90, 3*8);
//      display.setTextSize(1);      // Normal 1:1 pixel scale
//      display.print(UDP_packets);
      }
    }
    else {
      if ((millis() - last_packet_millis) > timeout_period) {
        UDP_packets_lost++;  
        UDP_packets_lost_total++; 
        last_packet_millis = millis();
//      display.setCursor(32, 3*8-1);
//      display.setTextSize(1);      // Normal 1:1 pixel scale
//      display.print(UDP_packets_lost_total);

        if (UDP_packets_lost>0 && UDP_packets_lost < TIMEOUT_TICKER_WIDTH){
          //for (uint8 i = 0; i <= UDP_packets_lost; i++) {
          //  display.drawPixel(TIMEOUT_TICKER_X+i,TIMEOUT_TICKER_Y,SSD1306_WHITE);
          //}
          display.drawPixel(TIMEOUT_TICKER_X+UDP_packets_lost,TIMEOUT_TICKER_Y,SSD1306_WHITE);
        }
      }
    }

}
