#include <WiFi.h>
#include <WebServer.h>
#define DECODE_RC5
#include <IRremote.hpp>

// --- Configuration ---
const char *ssid = "BMS_Master_AP";
const char *password = "12345678";
const int IR_RX_PIN = 15;
const int IR_TX_PIN = 4;
const int TOTAL_NODES = 32;
const unsigned long LIS_WINDOW = 600;      // Time to wait for all 9 IR packets
const unsigned long OFFLINE_TIMEOUT = 60000; // 1 minute stale threshold

WebServer server(80);

enum NodeStatus { OFFLINE, ONLINE, INCOMPLETE, CHECKSUM_ERROR };

struct BatteryCell {
    float voltage = 0.0;
    uint16_t auxData = 0;
    NodeStatus status = OFFLINE;
    unsigned long lastUpdateTime = 0;
    
    // Decoding buffers
    uint8_t receivedChunks[8];
    bool chunkPresent[8];
    uint8_t receivedChecksum = 0xFF;

    // Reliability
    uint32_t totalPolls = 0;
    uint32_t successfulPolls = 0;
};

BatteryCell pack[TOTAL_NODES];
int cycleCounter = 0;

// --- Helper Functions ---

String statusToString(NodeStatus s) {
    switch(s) {
        case ONLINE: return "ONLINE";
        case INCOMPLETE: return "PARTIAL";
        case CHECKSUM_ERROR: return "CRC_ERR";
        default: return "OFFLINE";
    }
}

// Optimized JSON Delivery
void handleData() {
    String json;
    json.reserve(2048);
    json = "{ \"nodes\": [";
    for (int i = 0; i < TOTAL_NODES; i++) {
        json += "{\"id\":" + String(i);
        json += ",\"v\":" + String(pack[i].voltage, 3);
        json += ",\"aux\":" + String(pack[i].auxData);
        json += ",\"status\":\"" + statusToString(pack[i].status) + "\"";
        json += ",\"rel\":" + String(pack[i].totalPolls > 0 ? (pack[i].successfulPolls * 100.0 / pack[i].totalPolls) : 0, 0);
        json += "}";
        if (i < TOTAL_NODES - 1) json += ",";
    }
    json += "], \"cycle\":" + String(cycleCounter) + "}";
    server.send(200, "application/json", json);
}

// Professional UI with AJAX updates
void handleRoot() {
    String html;
    html.reserve(4096);
    html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
    html += "<title>IR-BMS Master</title><script src='https://cdn.tailwindcss.com'></script>";
    html += "<script>";
    html += "function update(){ fetch('/data').then(r=>r.json()).then(data=>{";
    html += "  data.nodes.forEach(n=>{";
    html += "    const el=document.getElementById('n'+n.id);";
    html += "    if(el){ el.innerText = n.status==='ONLINE' ? n.v.toFixed(2)+'V' : n.status; }";
    html += "  });";
    html += "  document.getElementById('cyc').innerText = data.cycle;";
    html += "}); } setInterval(update, 2000);";
    html += "</script></head>";
    html += "<body class='bg-gray-950 text-white p-8'>";
    html += "<h1 class='text-2xl font-bold mb-6 text-teal-400'>IR-BMS Dashboard <span id='cyc' class='text-gray-600 text-sm'></span></h1>";
    html += "<div class='grid grid-cols-4 md:grid-cols-8 gap-4'>";
    
    for(int i=0; i<TOTAL_NODES; i++) {
        html += "<div class='bg-gray-900 border border-gray-800 p-4 rounded text-center'>";
        html += "<div class='text-xs text-gray-500'>Cell " + String(i) + "</div>";
        html += "<div id='n" + String(i) + "' class='text-lg font-mono font-bold'>-</div>";
        html += "</div>";
    }
    
    html += "</div></body></html>";
    server.send(200, "text/html", html);
}

// --- IR Logic ---

void processIncomingFragment(int nodeID, uint8_t rc5Cmd) {
    if (rc5Cmd & 0x20) { // Data Chunk
        uint8_t index = (rc5Cmd >> 2) & 0x07;
        uint8_t payload = rc5Cmd & 0x03;
        if (index < 8) {
            pack[nodeID].receivedChunks[index] = payload;
            pack[nodeID].chunkPresent[index] = true;
        }
    } else if (rc5Cmd & 0x10) { // Checksum Chunk
        pack[nodeID].receivedChecksum = rc5Cmd & 0x0F;
    }
}

bool pollNodeInternal(int nodeID, uint8_t pollCmd) {
    for (int j = 0; j < 8; j++) pack[nodeID].chunkPresent[j] = false;
    pack[nodeID].receivedChecksum = 0xFF;
    pack[nodeID].totalPolls++;

    IrSender.sendRC5(nodeID, pollCmd, 0);

    unsigned long startListen = millis();
    while (millis() - startListen < LIS_WINDOW) {
        if (IrReceiver.decode()) {
            if (IrReceiver.decodedIRData.protocol == RC5 && IrReceiver.decodedIRData.address == nodeID) {
                processIncomingFragment(nodeID, IrReceiver.decodedIRData.command);
            }
            IrReceiver.resume();
        }
        server.handleClient(); // Keep web server responsive
        yield();
    }

    // Validation
    bool allPresent = true;
    uint8_t calcSum = 0;
    uint16_t data = 0;
    
    for (int j = 0; j < 8; j++) {
        if (!pack[nodeID].chunkPresent[j]) allPresent = false;
        data |= (uint16_t)pack[nodeID].receivedChunks[j] << (j * 2);
        calcSum += pack[nodeID].receivedChunks[j];
    }

    if (allPresent && (calcSum & 0x0F) == pack[nodeID].receivedChecksum) {
        if (pollCmd == 0x01) pack[nodeID].voltage = data / 1000.0f;
        else if (pollCmd == 0x02) pack[nodeID].auxData = data;
        
        pack[nodeID].status = ONLINE;
        pack[nodeID].lastUpdateTime = millis();
        pack[nodeID].successfulPolls++;
        return true;
    }

    pack[nodeID].status = allPresent ? CHECKSUM_ERROR : INCOMPLETE;
    return false;
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    
    IrReceiver.begin(IR_RX_PIN, DISABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN, DISABLE_LED_FEEDBACK);
    
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.begin();
    Serial.println("BMS Master Started");
}

void loop() {
    server.handleClient();
    
    for (int i = 0; i < TOTAL_NODES; i++) {
        // Only poll if it's currently online, OR every 10th cycle to rediscover offline nodes
        if (pack[i].status != OFFLINE || (cycleCounter % 10 == 0)) {
            uint8_t cmd = (cycleCounter % 15 == 0) ? 0x02 : 0x01; // Occasionally poll Aux
            pollNodeInternal(i, cmd);
        }

        // Timeout check
        if (pack[i].status != OFFLINE && (millis() - pack[i].lastUpdateTime > OFFLINE_TIMEOUT)) {
            pack[i].status = OFFLINE;
        }
        
        yield(); 
    }
    cycleCounter++;
}
