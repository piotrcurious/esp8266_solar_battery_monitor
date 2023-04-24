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

//WIFI and UDP multicast includes
#include <WiFi.h>
#include <WiFiAP.h>
//#include <WiFiUdp.h>
//wifi config
const char* ssid = "sensor1";
const char* password = "testudp1234";
const char channel = 11; 
const char hidden = 0;
const char max_connection = 8; 
const bool ftm_responder = true; 
const int beacon_interval = 1000; 


#define AP_mode_on 1 ; // set to 1 to compile AP node


WiFiUDP Udp;
unsigned int multicastPort = 5683;  // local port to listen on
IPAddress multicastIP(224,0,1,187);

//char packetBuffer[1500];
//#pragma pack(push,1)
//typedef struct telemetry_frame {
// float voltage_ADC0 = 0;
//} tframe;
#include "telemetry_frame.h" // make sure to sync that with sender


#pragma pack(pop)
// end of WiFi

// OLED includes
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//tiny4koled includes 
//#include <Tiny4kOLED.h>
//#include "pzim3x5_font.h"  //extra tiny4koled fonts
//for small font printing with background overlay routines

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// end of OLED includes

//timer interrupt includes
// Select a Timer Clock
#define USING_TIM_DIV1                false           // for shortest and most accurate timer
#define USING_TIM_DIV16               false           // for medium time and medium accurate timer
#define USING_TIM_DIV256              true            // for longest timer but least accurate. Default

#include "ESP32_New_TimerInterrupt.h"
#include "ESP32_New_ISR_Timer.h"


// end of timer interrupt includes



// timer interrupt init 
#define HW_TIMER_INTERVAL_MS      10L

volatile uint32_t startMillis = 0;

// Init ESP32 timer 1
ESP32Timer ITimer(1);

// Init ISR timer 
ESP32_ISR_Timer ISR_Timer;
// end timer interrupt init

//timer interrupt helper functions
bool IRAM_ATTR TimerHandler(void * timerNo)
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  //  for(;;); // Don't proceed, loop forever
  }
  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  
  display.display();
  delay(1000); // Pause for 2 seconds
    // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

//  oled.begin(); 
  // tiny4koled begin

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(1000);

  // start timer interrupts
//    ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_MS * 1000, TimerHandler) ;
      for (uint16_t i = 0; i < NUMBER_ISR_TIMERS; i++)
  {
    ISR_Timer.setInterval(TimerInterval[i], irqCallbackFunc[i]);
  }

  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw white text on a black background
  // WiFi.begin(ssid, password);
//  WiFi.persistent(false); // no need to wear off the flash as we have all data in the sketch

#ifdef AP_mode_on // if ap mode , start and configure AP
  WiFi.softAP(ssid, password, channel, hidden, max_connection,ftm_responder, beacon_interval);
//  WiFi.softAP(ssid, password, channel, hidden, max_connection,ftm_responder);
  
 // WiFi.softAP(ssid, password);

    display.clearDisplay();
    display.display();
    delay(1000);
    
#endif AP_mode_on

#ifndef AP_mode_on // if not AP mode, start STA
///*  

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
//    delay(500);
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
//  display.clearDisplay();
  display.print("WiFi: ");
  display.print(millis());
  display.display();
    }
//  display.print(" OK");
//  display.setCursor(16,8);
//  display.print(WiFi.localIP());
  display.clearDisplay();
  display.display();   
//*/
#endif AP_mode_on 
 // Udp.begin(multicastPort);
  Udp.beginMulticast(multicastIP, multicastPort); // start multicast 

}

int UDP_packets = 0;
 
void loop() {
  struct telemetry_frame tframe ;
   //put your main code here, to run repeatedly:
//if (!display_refresh_sync) {
if (1){
  delay(20);
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw white text on a black background
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
//  display.clearDisplay();
  display.print(millis());
  display.display();

//tiny4koled 
// const DCfont *currentFont = FONT8X16DIGITS;
//  oled.setFont(currentFont);
//  oled.setFont(FONT8X16DIGITS);
//  oled.setFont(FONTPZIM3X5);
//  oled.setSpacing(FONTPZIM3X5->spacing + 1);
//  if (currentFont->width == 0) {
//    oled.setSpacing(1);
//  } else {
//    oled.setSpacing(currentFont->spacing + 1);
//  }
//  oled.setCursor(0, 1);
//  oled.print(millis());

//  oled.setCursor(0, 2);
//  oled.print(UDP_packets);

  display_refresh_sync=1; //set display_refresh_sync flag so we do not refresh unnessesairly
}
    int packetSize = Udp.parsePacket();
    if (packetSize){
    UDP_packets++;
    int message_size=Udp.read((byte*)&tframe,packetSize); // read the packet into the buffer

    if (message_size>0){
//      packetBuffer[message_size]='\0'; //// null terminate
 //     oled.setCursor(0,3);
 //     oled.print(packetBuffer);
      display.setCursor(0, 3*8);
      display.print(tframe.voltage_ADC0,3);
      display.print(F("V "));
      display.setCursor(64, 3*8);
      display.print(UDP_packets);

      display.display();
      }
    }

}
