#include <WiFi.h>
#include <WebServer.h>
#include <IRremote.hpp> // Ensure you are using the modern IRremote library (v4.x)

// --- Configuration ---
const char *ssid = "BMS_Master_AP";
const char *password = "12345678"; // Min 8 characters
const int IR_RX_PIN = 15;          // TSOP IR Receiver Out connected to GPIO15
const int IR_TX_PIN = 4;           // IR LED Driver circuit connected to GPIO4
const int TOTAL_NODES = 32;        // Tracking 32 individual battery nodes
const unsigned long LIS_WINDOW = 800; // ms to keep listening loop open for cell reply

WebServer server(80);

// Structure representing the state of each battery cell
struct BatteryCell {
    float voltage = 0.0;
    bool online = false;
    unsigned long lastUpdateTime = 0;
    
    // Decoding State buffers matching the node's protocol
    uint8_t receivedChunks[16];
    bool chunkPresent[16];
};

BatteryCell pack[TOTAL_NODES];

// --- Responsive Tailwind CSS Web Dashboard ---
String getDashboardHTML() {
    String html = "<!DOCTYPE html><html lang='en'><head>";
    html += "<meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>BMS Telemetry Dashboard</title>";
    html += "<script src='https://cdn.tailwindcss.com'></script>";
    html += "<meta http-equiv='refresh' content='4'>"; // Refresh dashboard view every 4s
    html += "</head><body class='bg-slate-900 text-slate-100 min-h-screen font-sans'>";
    
    html += "<div class='container mx-auto px-4 py-8'>";
    html += "  <header class='flex flex-col md:flex-row justify-between items-center border-b border-slate-700 pb-6 mb-8'>";
    html += "    <div>";
    html += "      <h1 class='text-3xl font-extrabold text-transparent bg-clip-text bg-gradient-to-r from-teal-400 to-indigo-400'>BMS Master Control</h1>";
    html += "      <p class='text-slate-400 text-sm mt-1'>Network SSID: <span class='text-teal-400 font-semibold'>" + String(ssid) + "</span> | AP IP: <span class='text-teal-400 font-semibold'>" + WiFi.softAPIP().toString() + "</span></p>";
    html += "    </div>";
    
    // Calculate aggregate values
    float totalVoltage = 0.0;
    int activeCells = 0;
    for (int i = 0; i < TOTAL_NODES; i++) {
        if (pack[i].online) {
            totalVoltage += pack[i].voltage;
            activeCells++;
        }
    }
    
    html += "    <div class='flex gap-4 mt-4 md:mt-0'>";
    html += "      <div class='bg-slate-800 border border-slate-700 rounded-xl p-4 text-center min-w-[130px]'>";
    html += "        <span class='block text-[10px] text-slate-400 uppercase font-bold tracking-wider'>Active Nodes</span>";
    html += "        <span class='text-2xl font-bold text-teal-400'>" + String(activeCells) + " / " + String(TOTAL_NODES) + "</span>";
    html += "      </div>";
    html += "      <div class='bg-slate-800 border border-slate-700 rounded-xl p-4 text-center min-w-[130px]'>";
    html += "        <span class='block text-[10px] text-slate-400 uppercase font-bold tracking-wider'>Series Voltage</span>";
    html += "        <span class='text-2xl font-bold text-indigo-400'>" + String(totalVoltage, 3) + " V</span>";
    html += "      </div>";
    html += "    </div>";
    html += "  </header>";

    // Grid of all 32 node cells
    html += "  <div class='grid grid-cols-2 sm:grid-cols-4 md:grid-cols-6 lg:grid-cols-8 gap-4'>";
    for (int i = 0; i < TOTAL_NODES; i++) {
        bool isOnline = pack[i].online;
        float v = pack[i].voltage;
        
        String borderCol = "border-slate-800 bg-slate-800/40";
        String valCol = "text-slate-500";
        String statusDot = "bg-red-500 animate-pulse";
        
        if (isOnline) {
            statusDot = "bg-emerald-400";
            if (v >= 3.65) {
                borderCol = "border-emerald-500/30 bg-slate-800";
                valCol = "text-emerald-400";
            } else if (v >= 3.0) {
                borderCol = "border-amber-500/30 bg-slate-800";
                valCol = "text-amber-400";
            } else {
                borderCol = "border-red-500/30 bg-slate-800 animate-pulse";
                valCol = "text-red-400";
            }
        }
        
        html += "    <div class='border rounded-xl p-4 transition duration-200 " + borderCol + "'>";
        html += "      <div class='flex justify-between items-center mb-1'>";
        html += "        <span class='text-[10px] font-semibold uppercase tracking-wider text-slate-400'>Node " + String(i) + "</span>";
        html += "        <span class='h-2.5 w-2.5 rounded-full " + statusDot + "'></span>";
        html += "      </div>";
        
        if (isOnline) {
            html += "      <div class='text-lg font-bold tracking-tight " + valCol + "'>" + String(v, 3) + " V</div>";
            unsigned long age = (millis() - pack[i].lastUpdateTime) / 1000;
            html += "      <div class='text-[9px] text-slate-500 mt-0.5'>Age: " + String(age) + "s ago</div>";
        } else {
            html += "      <div class='text-md font-bold tracking-tight text-slate-600'>OFFLINE</div>";
            html += "      <div class='text-[9px] text-slate-600 mt-0.5'>Unreachable</div>";
        }
        html += "    </div>";
    }
    html += "  </div>";
    html += "</div></body></html>";
    return html;
}

void handleRoot() {
    server.send(200, "text/html", getDashboardHTML());
}

// --- Frame Processing Logic ---
// Maps wrapped 3-bit sequences (0 to 7 twice) back to 16 linear indexes
void processIncomingFragment(int nodeID, uint8_t rc5Cmd, int &packetsCount) {
    uint8_t seq3Bit = (rc5Cmd >> 2) & 0x07; // Extracted 3-bit sequence (0 to 7)
    uint8_t payload = rc5Cmd & 0x03;        // Extracted 2-bit payload (0 to 3)

    // Reconstruct linear array position (0 to 15) using incoming fragment progress count
    int mappedIndex = seq3Bit;
    if (packetsCount >= 7) { 
        mappedIndex = seq3Bit + 8;
    }

    if (mappedIndex >= 0 && mappedIndex < 16) {
        pack[nodeID].receivedChunks[mappedIndex] = payload;
        pack[nodeID].chunkPresent[mappedIndex] = true;
        packetsCount++;
    }
}

void pollNode(int nodeID) {
    // 1. Reset state tracking variables for the queried Node
    int packetsCount = 0;
    for (int j = 0; j < 16; j++) {
        pack[nodeID].chunkPresent[j] = false;
        pack[nodeID].receivedChunks[j] = 0;
    }

    // 2. Transmit Wake & Poll Command (Address = nodeID, Command = 0x01)
    // We send it twice to ensure waking node picks it up reliably from sleep
    IrSender.sendRC5(nodeID, 0x01, 1);
    delay(40);
    IrSender.sendRC5(nodeID, 0x01, 1);

    // 3. Keep Listening Window open
    unsigned long startListen = millis();
    while (millis() - startListen < LIS_WINDOW) {
        if (IrReceiver.decode()) {
            // Verify address, ensure it is RC5, and check if bit 5 (0x20) is marked as DATA
            if (IrReceiver.decodedIRData.protocol == RC5 && 
                IrReceiver.decodedIRData.address == nodeID && 
                (IrReceiver.decodedIRData.command & 0x20)) {
                
                processIncomingFragment(nodeID, IrReceiver.decodedIRData.command, packetsCount);
            }
            IrReceiver.resume(); // Ready to capture next fragment
        }
        server.handleClient(); // Keep the AP web server responsive during transactions
        yield();
    }

    // 4. Verify Frame Assembly Completeness
    bool allChunksPresent = true;
    for (int j = 0; j < 16; j++) {
        if (!pack[nodeID].chunkPresent[j]) {
            allChunksPresent = false;
            break;
        }
    }

    if (allChunksPresent) {
        // Reassemble 16 2-bit fragments back to 32-bit Float format
        uint32_t reassembledBits = 0;
        for (int s = 0; s < 16; s++) {
            reassembledBits |= ((uint32_t)pack[nodeID].receivedChunks[s] << (s * 2));
        }

        // Direct memory map to float
        float targetVoltage;
        memcpy(&targetVoltage, &reassembledBits, 4);

        // Sanity boundaries (0.0V - 10.0V) to prevent data corruption noise
        if (targetVoltage >= 0.0f && targetVoltage <= 10.0f) {
            pack[nodeID].voltage = targetVoltage;
            pack[nodeID].online = true;
            pack[nodeID].lastUpdateTime = millis();
        }
    } else {
        // Drop node offline if it has not responded successfully for more than 20 seconds
        if (pack[nodeID].online && (millis() - pack[nodeID].lastUpdateTime > 20000)) {
            pack[nodeID].online = false;
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Configure the Wi-Fi AP
    WiFi.softAP(ssid, password);
    Serial.println("\n=================================");
    Serial.println("ESP32 BMS AP Started!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.println("=================================");

    // Initialize IR Transmitter and Receiver
    IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN);

    // Bind web services
    server.on("/", handleRoot);
    server.begin();
    Serial.println("AP Web Server online.");
}

void loop() {
    server.handleClient(); // Yield slot to serve client browser requests

    // Sequential cycle scanning each of the 32 device slots
    for (int i = 0; i < TOTAL_NODES; i++) {
        pollNode(i);
        yield();
    }
}

