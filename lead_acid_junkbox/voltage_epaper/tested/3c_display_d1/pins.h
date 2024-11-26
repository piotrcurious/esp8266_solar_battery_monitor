/* ==================================================================
 * Pin Mapping Examples
 * ==================================================================
 */


// Example mapping for an ePaper display
/*
#define EPAPER_CS     PIN_D8 // Chip Select (GPIO15)
#define EPAPER_DC     PIN_D3 // Data/Command (GPIO0)
#define EPAPER_RESET  PIN_D4 // Reset (GPIO2)
#define EPAPER_BUSY   PIN_D2 // Busy signal (GPIO4)
#define EPAPER_SCLK   PIN_D5 // SPI Clock (GPIO14)
#define EPAPER_MOSI   PIN_D7 // SPI MOSI (GPIO13)
#define EPAPER_MISO   PIN_D6 // SPI MISO (GPIO12)

*/

/*
// Example mapping for I2C devices
#define I2C_SCL PIN_D1 // I2C Clock (GPIO5)
#define I2C_SDA PIN_D2 // I2C Data (GPIO4)
*/

// Example use of onboard LED
//#define ONBOARD_LED PIN_D4 // Onboard LED (GPIO2)

/* ==================================================================
 * Reverse Reflection Map (_MAP Defines)
 * ==================================================================
 */

// Left Column Pins
//#define PIN_RST    -1  // Reset pin (not usable in code)
#define PIN_RST_MAP   "D0_loop  " //connected to D0 to wake up the device
//#define PIN_A0     A0  // Analog input
#define PIN_A0_MAP    "A0  "
//#define PIN_D0     16  // GPIO16
#define PIN_D0_MAP    "RST_loop" // connected to RST 
//#define PIN_D5     14  // GPIO14 (SPI SCLK)
#define PIN_D5_MAP    "EPAPER_SCLK"
#define EPAPER_SCLK   PIN_D5 // SPI Clock (GPIO14)
//#define PIN_D6     12  // GPIO12 (SPI MISO)
#define PIN_D6_MAP    "EPAPER_BUSY"
#define EPAPER_BUSY   PIN_D6 // BUSY (GPIO12)
//#define PIN_D7     13  // GPIO13 (SPI MOSI)
#define PIN_D7_MAP    "EPAPER_MOSI"
#define EPAPER_MOSI   PIN_D7 // SPI MOSI (GPIO13)
//#define PIN_D8     15  // GPIO15 (SPI CS)
#define PIN_D8_MAP    "EPAPER_CS"
#define EPAPER_CS     PIN_D8 // Chip Select (GPIO15)
//#define PIN_3V3    -1  // 3.3V Power (not usable in code)
#define PIN_3V3_MAP   "3V3 "

// Right Column Pins

//#define PIN_TX     1   // GPIO1 (TX)
#define PIN_TX_MAP  "TX"    // transmit data
//#define PIN_RX     3   // GPIO3 (TX)
#define PIN_RX_MAP  "PWM"   // PWM to control device
#define PWM_PIN PIN_RX
//#define PIN_D1     5   // GPIO5 (SCL for I2C)
#define PIN_D1_MAP    "I2C_SCL"
#define I2C_SCL PIN_D1 // I2C Clock (GPIO5)
//#define PIN_D2     4   // GPIO4 (SDA for I2C)
#define PIN_D2_MAP    "I2C_SDA"
#define I2C_SDA PIN_D2 // I2C Data (GPIO4)
//#define PIN_D3     0   // GPIO0
#define PIN_D3_MAP    "EPAPER_DC"
#define EPAPER_DC     PIN_D3 // Data/Command (GPIO0)
//#define PIN_D4     2   // GPIO2 (onboard LED)
#define PIN_D4_MAP    "EPAPER_RESET/LED"
#define EPAPER_RESET  PIN_D4 // Reset (GPIO2)
//#define PIN_GND    -1  // Ground (not usable in code)
#define PIN_GND_MAP   "GND "
//#define PIN_5V     -1  // 5V Power (not usable in code)
#define PIN_5V_MAP    "5V  "
