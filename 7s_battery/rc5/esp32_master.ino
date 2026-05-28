#include <WiFi.h>
#include <WebServer.h>
#include <IRremote.hpp> // Use the 4.x version of IRremote

// --- Configuration ---
const char *ssid = "BMS_Master_AP";
const char *password = "12345678";
const int IR_RX_PIN = 15;
const int IR_TX_PIN = 4;
const int TOTAL_NODES = 32;

WebServer server(80);

struct BatteryCell {
    float voltage = 0.0;
    bool online = false;
    uint32_t rawData = 0;
    uint32_t receivedMask = 0;
    unsigned long lastUpdate = 0;
};

BatteryCell pack[TOTAL_NODES];

// --- Web Page Generation ---
String getDashboardHTML() {
    String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'><style>";
    html += "body { font-family: sans-serif; text-align: center; background: #f4f4f4; }";
    html += "table { margin: auto; border-collapse: collapse; background: white; box-shadow: 0 2px 5px rgba(0,0,0,0.1); }";
    html += "th, td { padding: 12px; border: 1px solid #ddd; }";
    html += "th { background: #007bff; color: white; }";
    html += ".online { color: green; font-weight: bold; }";
    html += ".offline { color: red; }";
    html += "</style><meta http-equiv='refresh' content='5'></head><body>";
    html += "<h1>Battery Pack Status</h1>";
    html += "<table><tr><th>ID</th><th>Voltage</th><th>Status</th></tr>";

    float totalV = 0;
    for (int i = 0; i < TOTAL_NODES; i++) {
        html += "<tr><td>" + String(i) + "</td>";
        if (pack[i].online) {
            html += "<td>" + String(pack[i].voltage, 3) + "V</td><td class='online'>OK</td>";
            totalV += pack[i].voltage;
        } else {
            html += "<td>---</td><td class='offline'>OFFLINE</td>";
        }
        html += "</tr>";
    }
    
    html += "</table><h3>Total Voltage: " + String(totalV, 2) + "V</h3>";
    html += "</body></html>";
    return html;
}

void handleRoot() {
    server.send(200, "text/html", getDashboardHTML());
}

// --- IR Logic ---
void pollNextNode() {
    static int currentNode = 0;
    
    // Reset target node data
    pack[currentNode].rawData = 0;
    pack[currentNode].receivedMask = 0;

    // Send Wake/Request (RC5: Address=currentNode, Command=0x01)
    IrSender.sendRC5(currentNode, 0x01, 1);
    
    unsigned long listenStart = millis();
    while (millis() - listenStart < 600) { // Listening window
        if (IrReceiver.decode()) {
            // Check if RC5 and matches our node, and is a DATA packet (Bit 5 set)
            if (IrReceiver.decodedIRData.protocol == RC5 && 
                IrReceiver.decodedIRData.address == currentNode && 
                (IrReceiver.decodedIRData.command & 0x20)) {
                
                uint8_t seq = (IrReceiver.decodedIRData.command >> 1) & 0x0F;
                uint8_t bitVal = IrReceiver.decodedIRData.command & 0x01;

                if (!(pack[currentNode].receivedMask & (1UL << seq))) {
                    pack[currentNode].rawData |= ((uint32_t)bitVal << seq);
                    pack[currentNode].receivedMask |= (1UL << seq);
                }
            }
            IrReceiver.resume();
        }
        server.handleClient(); // Keep web server responsive during IR listen
    }

    // Check if we got enough data (for 32-bit float, we need all bits)
    if (pack[currentNode].receivedMask == 0xFFFFFFFF) {
        memcpy(&pack[currentNode].voltage, &pack[currentNode].rawData, 4);
        pack[currentNode].online = true;
        pack[currentNode].lastUpdate = millis();
    } else {
        // Only mark offline if we haven't heard from it in 30 seconds
        if (millis() - pack[currentNode].lastUpdate > 30000) {
            pack[currentNode].online = false;
        }
    }

    currentNode = (currentNode + 1) % TOTAL_NODES;
}

void setup() {
    Serial.begin(115200);
    
    // Setup AP Mode
    WiFi.softAP(ssid, password);
    Serial.println("AP Started. IP: " + WiFi.softAPIP().toString());

    // Setup IR
    IrReceiver.begin(IR_RX_PIN, ENABLE_LED_FEEDBACK);
    IrSender.begin(IR_TX_PIN);

    // Setup Web Server
    server.on("/", handleRoot);
    server.begin();
}

void loop() {
    server.handleClient();
    pollNextNode();
}
