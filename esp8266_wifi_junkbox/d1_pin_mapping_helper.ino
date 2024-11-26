/*
 * ==================================================================
 * ESP8266 D1 Mini Board Pin Layout
 * ==================================================================
 *
 * Physical Pin Layout (Vertical View, Top of Board at Top)
 *
 *          +----------------+
 *   3V3    | [3V3]   [RST] |  Reset (RST)
 *   GPIO16 | [D0 ]   [A0 ] |  Analog Input (0-1V)
 *   GPIO5  | [D1 ]   [G   ]|  Ground (G)
 *   GPIO4  | [D2 ]   [5V  ]|  5V Power
 *   GPIO0  | [D3 ]   [D8 ] |  GPIO15 (SPI CS)
 *   GPIO2  | [D4 ]   [D7 ] |  GPIO13 (SPI MOSI)
 *   GPIO14 | [D5 ]   [D6 ] |  GPIO12 (SPI MISO)
 *   GND    | [GND]   [3V3] |  3.3V Power
 *          +----------------+
 */

/* ==================================================================
 * Pin Assignment Definitions
 * ==================================================================
 */

// Left Column Pins
#define PIN_3V3    -1  // 3.3V Power (not usable in code)
#define PIN_D0     16  // GPIO16
#define PIN_D1     5   // GPIO5 (SCL for I2C)
#define PIN_D2     4   // GPIO4 (SDA for I2C)
#define PIN_D3     0   // GPIO0
#define PIN_D4     2   // GPIO2 (onboard LED)
#define PIN_D5     14  // GPIO14 (SPI SCLK)
#define PIN_GND    -1  // Ground (not usable in code)

// Right Column Pins
#define PIN_RST    -1  // Reset pin (not usable in code)
#define PIN_A0     A0  // Analog input
#define PIN_G      -1  // Ground (not usable in code)
#define PIN_5V     -1  // 5V Power (not usable in code)
#define PIN_D8     15  // GPIO15 (SPI CS)
#define PIN_D7     13  // GPIO13 (SPI MOSI)
#define PIN_D6     12  // GPIO12 (SPI MISO)
#define PIN_3V3B   -1  // 3.3V Power (duplicate, not usable in code)

/* ==================================================================
 * Pin Mapping Examples
 * ==================================================================
 */

// Example mapping for an ePaper display
#define EPAPER_CS     PIN_D8 // Chip Select (GPIO15)
#define EPAPER_DC     PIN_D3 // Data/Command (GPIO0)
#define EPAPER_RESET  PIN_D4 // Reset (GPIO2)
#define EPAPER_BUSY   PIN_D2 // Busy signal (GPIO4)
#define EPAPER_SCLK   PIN_D5 // SPI Clock (GPIO14)
#define EPAPER_MOSI   PIN_D7 // SPI MOSI (GPIO13)
#define EPAPER_MISO   PIN_D6 // SPI MISO (GPIO12)

// Example mapping for I2C devices
#define I2C_SCL PIN_D1 // I2C Clock (GPIO5)
#define I2C_SDA PIN_D2 // I2C Data (GPIO4)

// Example use of onboard LED
#define ONBOARD_LED PIN_D4 // Onboard LED (GPIO2)

/* ==================================================================
 * Setup and Loop
 * ==================================================================
 */
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("ESP8266 D1 Mini Pin Mapping Example");

  // Initialize onboard LED
  pinMode(ONBOARD_LED, OUTPUT);
  digitalWrite(ONBOARD_LED, LOW); // Turn off LED initially

  // Example: Initialize SPI pins for ePaper display
  pinMode(EPAPER_CS, OUTPUT);
  pinMode(EPAPER_DC, OUTPUT);
  pinMode(EPAPER_RESET, OUTPUT);

  // Example: Initialize I2C pins
  pinMode(I2C_SCL, INPUT_PULLUP);
  pinMode(I2C_SDA, INPUT_PULLUP);

  // Example: Reset ePaper display
  digitalWrite(EPAPER_RESET, LOW);
  delay(100);
  digitalWrite(EPAPER_RESET, HIGH);
}

void loop() {
  // Blink onboard LED as a basic example
  digitalWrite(ONBOARD_LED, HIGH); // Turn LED on
  delay(500);
  digitalWrite(ONBOARD_LED, LOW);  // Turn LED off
  delay(500);
}
