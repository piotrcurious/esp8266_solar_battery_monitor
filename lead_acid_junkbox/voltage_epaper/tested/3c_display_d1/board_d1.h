/*
 * ==================================================================
 * ESP8266 D1 Mini Board Pin Layout
 * ==================================================================
 *
 * Physical Pin Layout (Vertical View, Top of Board at Top)
 *
 *          +----------------+
 *   RST    | [RST]   [TX]   |  GPIO1 
 *   Analog | [A0 ]   [RX]   |  GPIO3
 *   GPIO16 | [D0 ]   [D1]   |  GPIO5 SCL
 *   GPIO14 | [D5 ]   [D2]   |  GPIO4 SDA
 *   GPIO12 | [D6 ]   [D3 ]  |  GPIO0 
 *   GPIO13 | [D7 ]   [D4 ]  |  GPIO2 
 *   GPIO15 | [D8 ]   [GND]  |  GND
 *   3.3V   | [3V3]   [5V]   |  5V Power
 *          +----------------+
 */

/* ==================================================================
 * Pin Assignment Definitions
 * ==================================================================
 */

// Left Column Pins
#define PIN_RST    -1  // Reset pin (not usable in code)
//#define PIN_A0     A0  // Analog input
#define PIN_D0     16  // GPIO16
#define PIN_D5     14  // GPIO14 (SPI SCLK)
#define PIN_D6     12  // GPIO12 (SPI MISO)
#define PIN_D7     13  // GPIO13 (SPI MOSI)
#define PIN_D8     15  // GPIO15 (SPI CS)
#define PIN_3V3    -1  // 3.3V Power (not usable in code)

// Right Column Pins

#define PIN_TX     1   // GPIO1 (TX)
#define PIN_RX     3   // GPIO3 (TX)
#define PIN_D1     5   // GPIO5 (SCL for I2C)
#define PIN_D2     4   // GPIO4 (SDA for I2C)
#define PIN_D3     0   // GPIO0
#define PIN_D4     2   // GPIO2 (onboard LED)
#define PIN_GND    -1  // Ground (not usable in code)
#define PIN_5V     -1  // 5V Power (not usable in code)
