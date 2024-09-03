//The fix is fake hallucination. beware.


#include "wifi_settings.h"
#include "telemetry_frame.h"
#include "user_interface.h"

const int analogInPin = A0;
#define LED_PIN 2
#define MODE_PIN 2

#ifdef PERSISTENT_WIFI
struct WIFI_SETTINGS_T _settings;
#endif

bool initialConnectionEstablished = false;
WiFiUDP udp;

// ... (previous code remains the same)

bool sendUDPPacket(const telemetry_frame& tframe) {
  if (udp.beginPacketMulticast(broadcast, port, WiFi.localIP())) {
    size_t bytesWritten = udp.write((uint8_t*)&tframe, sizeof(tframe));
    if (bytesWritten == sizeof(tframe)) {
      if (udp.endPacket()) {
        // Wait for the packet to be sent
        unsigned long startTime = millis();
        while (millis() - startTime < 50) { // Maximum wait time of 50ms
          if (wifi_get_tx_pending() == 0) {
            // No packets pending in the transmit buffer
            return true;
          }
          yield(); // Allow the ESP8266 to handle background tasks
        }
        // If we've waited too long, assume there was an issue
        return false;
      }
    }
  }
  return false;
}

void loop() {
  if (!initialConnectionEstablished) {
    initialConnectionEstablished = initialConnectAndStoreParams();
    if (!initialConnectionEstablished) {
      delay(5000);  // Wait before retrying
      return;
    }
  }

  telemetry_frame tframe;
  tframe.voltage_ADC0 = analogRead(analogInPin) * (17.0/1024);

  if (!sendUDPPacket(tframe)) {
    Serial.println("Failed to send UDP packet");
    // Optionally, implement retry logic here
  }

  if (digitalRead(MODE_PIN) == HIGH) {
    // ... (sleep code remains the same)
  } else {
    yield(); // Allow for background processing
  }
}
