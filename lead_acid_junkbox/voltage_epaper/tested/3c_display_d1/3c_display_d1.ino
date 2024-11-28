#include <GxEPD2_BW.h>       // Include GxEPD2 library for ePaper
#include <GxEPD2_3C.h>       // Tri-color ePaper support
#include <Adafruit_GFX.h>    // Adafruit GFX library for font support

//#include "open_sans_quick_16.h"
//#define VOLTAGE_FONT &Open_Sans_Light_16

//#include <Fonts/FreeSansBold18pt7b.h>
//#define VOLTAGE_FONT &FreeSansBold18pt7b

//#include "Special_Elite_Regular_48.h"
//#define VOLTAGE_FONT &Special_Elite_Regular_48

#include "Special_Elite_Regular_64.h"
#define VOLTAGE_FONT &Special_Elite_Regular_64

#define VOLTAGE_X 0
#define VOLTAGE_Y 64

#include "board_d1.h" // custom board pin definitions
#include "pins.h"     // pins definition

// Define display and pins for 2.13-inch tri-color ePaper
//#define MAX_DISPLAY_BUFFER_SIZE 8192 // Adjust if needed based on RAM limitations

//GxEPD2_3C<GxEPD2_213c, MAX_DISPLAY_BUFFER_SIZE> display(GxEPD2_213c(/*CS*/ 15, /*DC*/ 4, /*RST*/ 5, /*BUSY*/ 16));
//GxEPD2_3C<GxEPD2_213c, MAX_DISPLAY_BUFFER_SIZE> display(GxEPD2_213c(/*CS*/ 15, /*DC*/ 4, /*RST*/ 5, /*BUSY*/ 16));

GxEPD2_3C<GxEPD2_213_Z98c, GxEPD2_213_Z98c::HEIGHT> \
            display(GxEPD2_213_Z98c(EPAPER_CS, EPAPER_DC, EPAPER_RESET, EPAPER_BUSY)); // GDEY0213Z98 122x250, SSD1680

uint32_t lastRefreshTime = 0 ;
#define REFRESH_INTERVAL 1000*40 // define refresh interval 
#define LOW_VOLTAGE_SLEEP 25 // define how long to sleep when voltage is low 

//GxEPD2_BW<GxEPD2_213_B74, GxEPD2_213_B74::HEIGHT> display(GxEPD2_213_B74(/*CS=D8*/ 15, /*DC=D3*/ 0, /*RST=D4*/ 2, /*BUSY=D2*/ 4)); // GDEM0213B74 122x250, SSD1680

const int gpioPin = PWM_PIN ;  // GPIO10 (S3) to control
const int adcPin = PIN_A0;  // Pin to measure voltage (use a voltage divider to scale 12V to ESP8266 ADC range)
const float max_voltage = 17.0;  // Adjust based on your voltage divider 
const float highThreshold = 14.0;       // Voltage threshold to stay awake
const float lowThreshold = 13.8;        // Voltage threshold to go back to deep sleep
const float redThreshold = 12.8;        // Display red for voltages below this

// Define a struct to store data
struct RTCData {
  uint32_t bootCount = 0 ; // Example: Incremental boot counter
  uint32_t lastMillis; // Last millis
};

// Create an instance of the struct
RTCData rtcData;

// Define the RTC memory range
#define RTC_START_ADDRESS 64 // Start address (avoid first 64 bytes used by system)

// Function to write struct to RTC memory
bool saveRTCData() {
  ESP.rtcUserMemoryWrite(RTC_START_ADDRESS, (uint32_t*)&rtcData, sizeof(rtcData));
  return true; // You can add error checking if needed
}

// Function to read struct from RTC memory
bool loadRTCData() {
  return ESP.rtcUserMemoryRead(RTC_START_ADDRESS, (uint32_t*)&rtcData, sizeof(rtcData));
}

uint32_t millisOffset ; 

void setMillis(unsigned long new_millis){
  noInterrupts();
 millisOffset = new_millis;
  interrupts();
}

uint32 getMillis() {
 return millis()+millisOffset;
}
void setup() {
  Serial.flush();  
  Serial.begin(115200);
//  Serial.begin(74880);

  pinMode(gpioPin, OUTPUT);
  digitalWrite(gpioPin, LOW);  // Start with gpio off
  
  if (loadRTCData()) {
    Serial.println("RTC data loaded successfully!");
    Serial.printf("Boot count: %u\n", rtcData.bootCount);
    Serial.printf("Last millis: %u\n", rtcData.lastMillis);

    // Increment boot count
    rtcData.bootCount++;
    setMillis(rtcData.lastMillis);
    
    // Update other data (example)
  } else {
    Serial.println("Failed to load RTC data. ");
  }
  Serial.flush();  

  // Initialize the ePaper display
  display.init(115200);
  display.setRotation(1);  // Rotate for landscape view
//  display.setPartialWindow(0, 0, 64,64); 
  
  display.setTextColor(GxEPD_BLACK); // Default to black text
//  display.setFont(&FreeSansBold18pt7b); // Set larger font from Adafruit GFX library
  display.setFont(VOLTAGE_FONT); // Set larger font from Adafruit GFX library

//  display.fillScreen(GxEPD_WHITE); // Clear display with white background
  //display.display(); // Apply changes
}

void loop() {
  yield();// for WDT
  float batteryVoltage = readBatteryVoltage();

  Serial.print("Battery Voltage: ");
  Serial.println(batteryVoltage);
  displayVoltage(batteryVoltage); // Display the current voltage on ePaper

  if (getMillis() - lastRefreshTime > REFRESH_INTERVAL) {
      display.refresh();  // Refresh the ePaper display to show changes
      lastRefreshTime = millis();
      }

  if (batteryVoltage > highThreshold) {
    Serial.println("Voltage is above 14V, staying awake...");
    digitalWrite(gpioPin, HIGH);  // Turn on GPIO4
    
    // Monitor the voltage every 60 seconds
    if (batteryVoltage > lowThreshold) {
      //delay(60000);  // Wait for 60 seconds
      batteryVoltage = readBatteryVoltage();
      Serial.print("Monitoring Voltage: ");
      Serial.println(batteryVoltage);
      displayVoltage(batteryVoltage); // Update voltage on ePaper
    } else {
    // Once voltage falls below 13.8V, turn off GPIO4 //and go back to sleep
    Serial.println("Voltage dropped below 13.8V, turning off GPIO4");
    digitalWrite(gpioPin, LOW);
 //   goToDeepSleep();
    }
  } else {
    if (batteryVoltage < lowThreshold) {    // for hysteresis
    Serial.println("Voltage is below 13.8V, going to deep sleep...");
    digitalWrite(gpioPin, LOW);
//    delay(5000);
    goToDeepSleep(LOW_VOLTAGE_SLEEP);
    } 
  }
}

// Function to display battery voltage on ePaper
void displayVoltage(float voltage) {
  display.fillScreen(GxEPD_WHITE);  // Clear the display
  // Set text color based on voltage threshold
  if (voltage < redThreshold) {
    display.setTextColor(GxEPD_RED);  // Display red text for voltages below 12.8V
 //   display.setTextColor(GxEPD_BLACK);  // Display red text for voltages below 12.8V

  } else {
    display.setTextColor(GxEPD_BLACK);  // Display black text for voltages above 12.8V
  }

  // Display the voltage on ePaper
  display.setCursor(VOLTAGE_X, VOLTAGE_Y);  // Adjust position if necessary
//  display.print("Voltage: ");
  display.print(voltage, 2);  // Print voltage with 2 decimal places
  display.println("V");

    display.display();  // Refresh the ePaper display to show changes


}

// Function to read the battery voltage from ADC
float readBatteryVoltage() {
  int analogValue = analogRead(adcPin);  // Read from A0 pin
  float voltage = analogValue * (max_voltage/1024.0);  // Convert ADC reading to voltage
  return voltage;
}

// Function to go to deep sleep
void goToDeepSleep(uint16_t sleep_period) {
  Serial.flush();
  display.hibernate();  // Put ePaper display in low-power mode
  // Save updated data back to RTC memory
  rtcData.lastMillis=getMillis()+sleep_period*1000;
  if (saveRTCData()) {
    Serial.println("RTC data saved successfully!");
  } else {
    Serial.println("Failed to save RTC data.");
  }
  ESP.deepSleep(sleep_period*1000000);  // Sleep for 10 seconds (in microseconds)
}
