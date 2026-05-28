#include <IRremote.h>

const int RECV_PIN = 2;
const int SEND_PIN = 3; // Standard PWM pin for IR LED
const int TOTAL_NODES = 32;
const unsigned long NODE_TIMEOUT = 800; // ms to wait for all fragments

IRsend irsend;
IRrecv irrecv(RECV_PIN);
decode_results results;

// Structure to track the state of each battery cell
struct BatteryCell {
  float voltage;
  bool online;
  uint32_t rawData;
  uint16_t receivedMask; // Bitmask to track which of the 16 fragments arrived
};

BatteryCell pack[TOTAL_NODES];

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
  Serial.println("--- BMS Master Initialized ---");
}

// Function to clear a cell's buffer before polling
void resetCellBuffer(int id) {
  pack[id].rawData = 0;
  pack[id].receivedMask = 0;
  pack[id].online = false;
}

void pollNode(int nodeID) {
  resetCellBuffer(nodeID);
  
  Serial.print("Querying Cell ");
  Serial.print(nodeID);
  Serial.print("... ");

  // Send Command 0x01 (Request Voltage) to specific Node Address
  // We send it twice to ensure the node wakes up from deep sleep
  irsend.sendRC5(nodeID, 0x01, 1);
  delay(50);
  irsend.sendRC5(nodeID, 0x01, 1);
  
  irrecv.enableIRIn(); // Re-enable receiver after sending

  unsigned long startTime = millis();
  int fragmentsCaptured = 0;

  // Listen for fragments
  while (millis() - startTime < NODE_TIMEOUT) {
    if (irrecv.decode(&results)) {
      // Check if the data is from the node we queried and is a DATA packet (Bit 5 set)
      if (results.address == nodeID && (results.value & 0x20)) {
        
        // Protocol: [Bit 5: 1][Bits 4-1: Sequence (0-15)][Bit 0: Data Bit]
        // Note: For simplicity here, we use 1 bit per packet = 32 packets
        // If using 2 bits per packet, adjust the mask/shift accordingly.
        
        uint8_t seq = (results.value >> 1) & 0x0F;
        uint8_t bitVal = results.value & 0x01;

        if (!(pack[nodeID].receivedMask & (1 << seq))) {
          pack[nodeID].rawData |= ((uint32_t)bitVal << seq);
          pack[nodeID].receivedMask |= (1 << seq);
          fragmentsCaptured++;
        }
      }
      irrecv.resume();
    }
    
    // If we have all 32 fragments (for a 32-bit float), we can stop early
    if (fragmentsCaptured == 32) break;
  }

  if (fragmentsCaptured > 0) {
    // Reassemble raw bits into float
    memcpy(&pack[nodeID].voltage, &pack[nodeID].rawData, 4);
    pack[nodeID].online = true;
    Serial.print(pack[nodeID].voltage);
    Serial.println("V");
  } else {
    Serial.println("TIMEOUT (OFFLINE)");
  }
}

void displayPackStatus() {
  Serial.println("\n=== Battery Pack Report ===");
  Serial.println("ID\tVoltage\tStatus");
  float totalVoltage = 0;
  
  for (int i = 0; i < TOTAL_NODES; i++) {
    Serial.print(i);
    Serial.print("\t");
    if (pack[i].online) {
      Serial.print(pack[i].voltage);
      Serial.println("V\tOK");
      totalVoltage += pack[i].voltage;
    } else {
      Serial.println("---\tOFFLINE");
    }
  }
  Serial.print("Total Pack Voltage: ");
  Serial.print(totalVoltage);
  Serial.println("V\n");
}

void loop() {
  for (int i = 0; i < TOTAL_NODES; i++) {
    pollNode(i);
    delay(100); // Small cooldown between nodes
  }
  
  displayPackStatus();
  delay(10000); // Wait 10 seconds before next full pack scan
}
