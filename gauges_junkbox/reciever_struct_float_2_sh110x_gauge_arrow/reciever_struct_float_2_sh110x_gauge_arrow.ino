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

#include "display_settings.h" // includes configuration of on-screen widgets
uint8_t  display_brightness ; // global variable to adjust display brightness

#include "arrow_gauge.h" // include arrow gauge settings and variables
#include "arrow_gauge.cpp" //include arrow gauge functions

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
IRAM_ATTR void display_interface_interrupt(){
 display_refresh_sync=0; //clear display_refresh_sync at regular intervals
}

IRAM_ATTR void UDP_interface_interrupt(){

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

#ifdef SCREENSAVER_TRICKS
uint8_t screensaver_x_offset = 0 ;
uint8_t screensaver_y_offset = 0 ;
#endif //SCREENSAVER_TRICKS

uint8_t screen = 2 ; //choose screen to use 
                     //  1 - voltage display by fading text
                     //  2 - voltage display by arrow gauge with min/max


//arrow gauge

#define BUFFER_SIZE 128
#define ABS_MIN 0.0 // change this to your absolute minimum value
#define ABS_MAX 10.0 // change this to your absolute maximum value
float buffer[BUFFER_SIZE]; // circular buffer to store the last 128 values
int gauge_index = 0; // index to keep track of the buffer position
float gauge_min = ABS_MAX; // minimum value in the buffer
float gauge_max = ABS_MIN; // maximum value in the buffer


void updateMinMax() {
  // this function should update the min and max values based on the buffer contents
  // we use a simple linear search algorithm
  gauge_min = ABS_MAX;
  gauge_max = ABS_MIN;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    if (buffer[i] < gauge_min) {
      gauge_min = buffer[i];
    }
    if (buffer[i] > gauge_max) {
      gauge_max = buffer[i];
    }
  }
}

void displayValue(float value) {
  // this function should display the value as a vertical arrow pointing at a scale
  // we use the Adafruit GFX library to draw the graphics
  display.clearDisplay(); // clear the display
  display.setTextSize(1); // set the text size to 1
  display.setTextColor(SSD1306_WHITE); // set the text color to white
  display.setCursor(0, 0); // set the cursor position to the top left corner
  display.print(gauge_min,2); // print the minimum value
  display.setCursor(SCREEN_WIDTH - 24, 0); // set the cursor position to the top right corner
  display.print(gauge_max,2); // print the maximum value
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE); // draw a horizontal line for the scale
  int x = map(int(value*100),int(gauge_min*100),int(gauge_max*100), 2, SCREEN_WIDTH - 3); // map the value to the x coordinate

  display.drawLine(x, 10, 0, SCREEN_HEIGHT - 1, SSD1306_WHITE); // draw a vertical line for the arrow
  display.drawLine(x - 2, 12, x, 10, SSD1306_WHITE); // draw the left part of the arrow head
  display.drawLine(x + 2, 12, x, 10, SSD1306_WHITE); // draw the right part of the arrow head
//  display.display(); // update the display
}

void draw_gauge(float value) {
 buffer[gauge_index] = value; // store the value in the buffer
  gauge_index = (gauge_index + 1) % BUFFER_SIZE; // increment the index and wrap around if necessary
  updateMinMax(); // update the minimum and maximum values in the buffer
  displayValue(value); // display the value on the OLED screen
//  delay(100); // wait for 100 milliseconds
}


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
  delay(1000);

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
    // Clear the buffer
  display.clearDisplay();
  // Draw a single pixel in white aka "Ready"
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(500);

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
}//void setup() {

//---------------

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
  telemetry_frame tframe ; // use telemetry_frame struct from telemetry_frame.h

  if (!display_refresh_sync) {
//  display.setTextSize(1);      // Normal 1:1 pixel scale
//  display.setTextColor(SSD1306_WHITE,SSD1306_BLACK); // Draw white text on a black background
//  display.setCursor(0, 0);     // Start at top-left corner
//  display.cp437(true);         // Use full 256 char 'Code Page 437' font
//  display.print(millis());
   //   display.setCursor(10,3*8-1);
   //   display.setTextSize(1);
    //  display.print(WiFi.RSSI());

    if (display_brightness>0){
    display_brightness-- ;        // fade the display
    }
    if (display_brightness <=200) {
  //    display_brightness = 0; // min
      // screen saver
     for (uint8 i = 0; i <= 32; i++) {  // strength of the fade
     display.drawPixel(random(128),random(64),SSD1306_BLACK);
     }
     
  //display.drawFastHLine(0, random(64) , 127, SSD1306_BLACK);
  //display.drawFastVLine(random(128), 0 , 64, SSD1306_BLACK);
  
    }
//TODO: ssd/sh displays ifdefs
    display.setContrast(display_brightness);

//TODO: debug 
//      display.setCursor(0, 48);
//      display.setTextSize(1);  // small
//      display.print(display_brightness);
//      display.print(F("% "));

  display.display();
  display_refresh_sync=1; //set display_refresh_sync flag so we do not refresh unnessesairly
  }// if (!display_refresh_sync){

// TODO: this should go to UDP interrupt
       size_t packetSize = Udp.parsePacket();
       
#ifdef WIFI_BARS
       if ((millis() - wifi_bars_millis) > WIFI_BARS_PERIOD) {
#ifndef AP_mode_on 
        display_wifi_bars();
#endif //AP_mode_on
        wifi_bars_millis=millis();
       }
#endif 

    if (packetSize){
// packet accounting
    UDP_packets++;
    display_brightness = 255; 
    last_packet_millis = millis(); // reset last packet counter

#ifdef TIMEOUT_TICKER
        if (UDP_packets_lost>1){
          UDP_packets_lost--; 
          if (UDP_packets_lost>=TIMEOUT_TICKER_WIDTH){ UDP_packets_lost = TIMEOUT_TICKER_WIDTH;}
          for (uint8 i = 0; i <= UDP_packets_lost; i++) {
            display.drawPixel(TIMEOUT_TICKER_X+i,TIMEOUT_TICKER_Y,SSD1306_BLACK);
          }
        }
    UDP_packets_lost = (UDP_packets_lost+UDP_packets_lost)/2; // divide lost packets counter by two to create persistent average 
    if (UDP_packets_lost>0){
      display.drawPixel(TIMEOUT_TICKER_X+UDP_packets_lost,TIMEOUT_TICKER_Y,SSD1306_WHITE);
    }
#endif //TIMEOUT_TICKER
    
//    int message_size=Udp.read((byte*)&tframe,packetSize); // read the packet into the buffer
                                                          // TODO limit the amount of data read by size of the struct
                                                          // otherwise malformed packet can overwrite memory
                                                          
    int message_size=Udp.read((byte*)&tframe,sizeof(tframe)); // read the packet into the buffer
                                            //read only size of struct to avoid oversized packet overfill the memory


    if (message_size>0){
//      packetBuffer[message_size]='\0'; //// null terminate

      if (screen == 1){
#ifndef SCREENSAVER_TRICKS
          display.setCursor(0, 0);
#endif //SCREENSAVER_TRICKS
#ifdef SCREENSAVER_TRICKS
          display.clearDisplay(); //todo defines of offset constrainsts
          if (screensaver_x_offset<8) {screensaver_x_offset=screensaver_x_offset+random(2);} 
          if (screensaver_x_offset>0) {screensaver_x_offset=screensaver_x_offset-random(2);} 

          if (screensaver_y_offset<40) {screensaver_y_offset=screensaver_y_offset+random(2);} 
          if (screensaver_y_offset>0) {screensaver_y_offset=screensaver_y_offset-random(2);} 

          //display.setCursor(random(8),random(40));      
          display.setCursor(screensaver_x_offset,screensaver_y_offset);      
#endif//SCREENSAVER_TRICKS
          display.setTextSize(3);      // Normal 1:1 pixel scale
          display.print(tframe.voltage_ADC0,2);
          display.print(F("V "));
      }// if (screen == 1) { 

        if (screen == 2) {
//          display.setCursor(0, 0);
//          display.setTextSize(1);      // Normal 1:1 pixel scale
//          display.print(tframe.voltage_ADC0,2);
//          display.print(F("V "));

          draw_gauge(tframe.voltage_ADC0);
          
        }


#ifdef TIMEOUT_TICKER
//      display.setCursor(90, 3*8);
//      display.setTextSize(1);      // Normal 1:1 pixel scale
//      display.print(UDP_packets_lost);
#endif //TIMEOUT_TICKER
      }
      
    }
    else {
      if ((millis() - last_packet_millis) > timeout_period) {
        UDP_packets_lost_total++; 
        last_packet_millis = millis();
//      display.setCursor(32, 3*8-1);
//      display.setTextSize(1);      // Normal 1:1 pixel scale
//      display.print(UDP_packets_lost_total);

#ifdef TIMEOUT_TICKER
        if (UDP_packets_lost<TIMEOUT_TICKER_WIDTH) {UDP_packets_lost++;} 
        if (UDP_packets_lost>=TIMEOUT_TICKER_WIDTH){ UDP_packets_lost = TIMEOUT_TICKER_WIDTH;}
        if (UDP_packets_lost>0 && UDP_packets_lost < TIMEOUT_TICKER_WIDTH){
          //for (uint8 i = 0; i <= UDP_packets_lost; i++) {
          //  display.drawPixel(TIMEOUT_TICKER_X+i,TIMEOUT_TICKER_Y,SSD1306_WHITE);
          //}
          display.drawPixel(TIMEOUT_TICKER_X+UDP_packets_lost,TIMEOUT_TICKER_Y,SSD1306_WHITE);
        }//if (UDP_packets_lost>0 && UDP_packets_lost < TIMEOUT_TICKER_WIDTH){
#endif //TIMEOUT_TICKER
      }//if ((millis() - last_packet_millis) > timeout_period) {
    }//else {

}// void loop() {
