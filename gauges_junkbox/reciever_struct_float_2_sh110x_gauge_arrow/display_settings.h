// OLED includes
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define SCREENSAVER_TRICKS 1 // uncomment if you want to use various screen-saving tricks 
  // to reduce OLED burnout 
  // methods include:
  // - fading by erasing random pixels when no new packets are in
  // - dimming display when no new packets are in
  // - placing text randomly around
  // TODO: chaotically implemented. just a quick hack for 128x64 display. 
  
  
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
//#define SSD1306_SWITCHCAPVCC SH110X_DCDC
#define SSD1306_WHITE SH110X_WHITE
#define SSD1306_BLACK SH110X_BLACK
#define SSD1306_INVERSE SH110X_INVERSE

//#define TIMEOUT_TICKER  //define to display timeout ticker 
                        //the ticker has X offset indicating average of lost packets spanning maximum of it's width
#define TIMEOUT_TICKER_WIDTH  63 //and ticks for width defined here
#define TIMEOUT_TICKER_X      0 
#define TIMEOUT_TICKER_Y      31

#define WIFI_BARS //define to display signal strenght bars when in client mode
                  //TODO: EXCLUDE LOGIC NOT fully IMPLEMENTED
#define WIFI_BARS_X           127-10-64
#define WIFI_BARS_Y           8
#define WIFI_BARS_PERIOD      500 //update period of the bars
