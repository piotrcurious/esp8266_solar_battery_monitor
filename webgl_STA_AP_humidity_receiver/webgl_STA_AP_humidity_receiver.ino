#include "wifi_settings.h"
#include "telemetry_frame.hpp"
#include <ESPAsyncWebServer.h>
#include <AsyncUDP.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Preferences.h>

#define NUMBER_OF_BUFFERS 5
#define MINUTES_GRAPH_BUFFER_MAX 1440

// Buffers to store 24 hours of data (1 minute resolution)
float* minutes_buffer[NUMBER_OF_BUFFERS];
int buffer_head = 0; // Circular buffer head
uint32_t total_packets = 0;
telemetry_frame last_frame;
bool new_packet_received = false;

SemaphoreHandle_t bufferMutex = NULL;
Preferences prefs;

// Current STA settings
String current_sta_ssid;
String current_sta_password;

AsyncWebServer server(80);
AsyncUDP udp;

void loadSettings() {
    prefs.begin("wifi-config", true);
    current_sta_ssid = prefs.getString("ssid", sta_ssid);
    current_sta_password = prefs.getString("pass", sta_password);
    prefs.end();
}

void saveSettings(String ssid, String pass) {
    prefs.begin("wifi-config", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}

// HTML/JS/WebGL Frontend
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 WebGL Telemetry</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { background: #050505; color: #eee; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; margin: 0; overflow: hidden; }
        #controls { position: absolute; top: 10px; left: 10px; z-index: 10; background: rgba(20,20,20,0.85); padding: 15px; border-radius: 8px; border: 1px solid #444; backdrop-filter: blur(4px); }
        canvas { display: block; width: 100vw; height: 100vh; }
        .btn { background: #333; border: 1px solid #555; color: #ccc; padding: 8px 12px; cursor: pointer; margin: 4px; border-radius: 4px; transition: all 0.2s; font-size: 14px; min-width: 130px; text-align: left; display: flex; align-items: center; }
        .btn:hover { background: #444; border-color: #777; }
        .btn.active { background: #0056b3; color: white; border-color: #007bff; box-shadow: 0 0 10px rgba(0,123,255,0.4); }
        #info { position: absolute; bottom: 10px; left: 10px; font-size: 12px; opacity: 0.6; pointer-events: none; }
        .legend-color { display: inline-block; width: 12px; height: 12px; margin-right: 10px; border-radius: 2px; flex-shrink: 0; }
        .toggle-group { margin-top: 10px; padding-top: 10px; border-top: 1px solid #444; }

        /* Modal Styles */
        .modal { display: none; position: fixed; z-index: 100; left: 0; top: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.8); }
        .modal-content { background: #222; margin: 15% auto; padding: 20px; border: 1px solid #444; width: 300px; border-radius: 8px; }
        .modal-content h2 { margin-top: 0; font-size: 18px; }
        .modal-content input { width: 100%; padding: 8px; margin: 10px 0; background: #333; border: 1px solid #555; color: white; box-sizing: border-box; }
        .modal-content .actions { display: flex; justify-content: flex-end; gap: 10px; }
    </style>
</head>
<body>
    <div id="controls">
        <div style="margin-bottom:10px; font-weight:bold; color:#aaa; font-size:11px; text-transform:uppercase; letter-spacing:1px;">Channels</div>
        <button id="btn-combined" class="btn active" onclick="setMode(-1)"><span class="legend-color" style="background:white; border:1px solid #888"></span>Overlay View</button>
        <button id="btn-0" class="btn" onclick="setMode(0)"><span class="legend-color" style="background:red"></span>TX Active</button>
        <button id="btn-1" class="btn" onclick="setMode(1)"><span class="legend-color" style="background:lime"></span>Voltage</button>
        <button id="btn-2" class="btn" onclick="setMode(2)"><span class="legend-color" style="background:yellow"></span>RSSI</button>
        <button id="btn-3" class="btn" onclick="setMode(3)"><span class="legend-color" style="background:blue"></span>Humidity</button>
        <button id="btn-4" class="btn" onclick="setMode(4)"><span class="legend-color" style="background:white"></span>Temperature</button>

        <div class="toggle-group">
            <button id="btn-stack" class="btn" onclick="toggleStack()"><span class="legend-color" style="background:#888"></span>Stack Graphs</button>
            <button class="btn" onclick="openSettings()"><span class="legend-color" style="background:#555"></span>WiFi Settings</button>
        </div>
    </div>

    <div id="settingsModal" class="modal">
        <div class="modal-content">
            <h2>WiFi Settings</h2>
            <label>STA SSID</label>
            <input type="text" id="staSsid" placeholder="SSID">
            <label>STA Password</label>
            <input type="password" id="staPass" placeholder="Password">
            <div class="actions">
                <button class="btn" style="min-width:auto" onclick="closeSettings()">Cancel</button>
                <button class="btn active" style="min-width:auto" onclick="saveWifiSettings()">Save & Reboot</button>
            </div>
        </div>
    </div>

    <div id="info">Packets: <span id="pkt">0</span> | IPs: <span id="ips">...</span></div>
    <canvas id="glCanvas"></canvas>

    <script>
        const canvas = document.getElementById('glCanvas');
        const gl = canvas.getContext('webgl');
        let currentMode = -1;
        let isStacked = false;
        let telemetryData = new Float32Array(5 * 1440);

        const colors = [
            [1.0, 0.2, 0.2, 1.0], // Red
            [0.2, 1.0, 0.2, 1.0], // Green
            [1.0, 1.0, 0.2, 1.0], // Yellow
            [0.3, 0.6, 1.0, 1.0], // Blue
            [0.9, 0.9, 0.9, 1.0]  // White
        ];

        const vsSource = `
            attribute vec2 aPosition;
            uniform vec2 uScale;
            uniform vec2 uOffset;
            void main() {
                gl_Position = vec4(aPosition * uScale + uOffset, 0.0, 1.0);
            }
        `;

        const fsSource = `
            precision mediump float;
            uniform vec4 uColor;
            void main() {
                gl_FragColor = uColor;
            }
        `;

        function createShader(gl, type, source) {
            const shader = gl.createShader(type);
            gl.shaderSource(shader, source);
            gl.compileShader(shader);
            return shader;
        }

        const program = gl.createProgram();
        gl.attachShader(program, createShader(gl, gl.VERTEX_SHADER, vsSource));
        gl.attachShader(program, createShader(gl, gl.FRAGMENT_SHADER, fsSource));
        gl.linkProgram(program);
        gl.useProgram(program);

        const positionBuffer = gl.createBuffer();
        const aPosition = gl.getAttribLocation(program, 'aPosition');
        const uScale = gl.getUniformLocation(program, 'uScale');
        const uOffset = gl.getUniformLocation(program, 'uOffset');
        const uColor = gl.getUniformLocation(program, 'uColor');

        function setMode(m) {
            currentMode = m;
            updateButtons();
            draw();
        }

        function toggleStack() {
            isStacked = !isStacked;
            updateButtons();
            draw();
        }

        function updateButtons() {
            document.querySelectorAll('.btn').forEach(b => b.classList.remove('active'));
            if (currentMode === -1) document.getElementById('btn-combined').classList.add('active');
            else document.getElementById('btn-' + currentMode).classList.add('active');
            if (isStacked) document.getElementById('btn-stack').classList.add('active');
        }

        function openSettings() {
            document.getElementById('settingsModal').style.display = 'block';
        }

        function closeSettings() {
            document.getElementById('settingsModal').style.display = 'none';
        }

        async function saveWifiSettings() {
            const ssid = document.getElementById('staSsid').value;
            const pass = document.getElementById('staPass').value;
            const formData = new FormData();
            formData.append('ssid', ssid);
            formData.append('pass', pass);

            try {
                await fetch('/settings', { method: 'POST', body: formData });
                alert("Settings saved. Device will reboot.");
                closeSettings();
            } catch (e) {
                alert("Error saving settings.");
            }
        }

        async function fetchData() {
            try {
                const response = await fetch('/data');
                const buffer = await response.arrayBuffer();
                telemetryData = new Float32Array(buffer);

                const statusRes = await fetch('/status');
                const status = await statusRes.json();
                document.getElementById('pkt').innerText = status.packets;
                document.getElementById('ips').innerText = status.ips;

                draw();
            } catch (e) {}
        }

        function draw() {
            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
            gl.viewport(0, 0, canvas.width, canvas.height);
            gl.clearColor(0.01, 0.01, 0.01, 1.0);
            gl.clear(gl.COLOR_BUFFER_BIT);

            gl.enableVertexAttribArray(aPosition);
            gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
            gl.vertexAttribPointer(aPosition, 2, gl.FLOAT, false, 0, 0);

            const drawChannel = (idx, color, viewScale = [1, 1], viewOffset = [0, 0]) => {
                const offset = idx * 1440;
                let min = Infinity, max = -Infinity;
                for (let i = 0; i < 1440; i++) {
                    let v = telemetryData[offset + i];
                    if (isNaN(v)) continue;
                    if (v < min) min = v;
                    if (v > max) max = v;
                }
                if (min === Infinity) return;
                if (max === min) { max += 0.1; min -= 0.1; }

                let points = [];
                const flush = () => {
                    if (points.length >= 4) {
                        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(points), gl.STATIC_DRAW);
                        gl.uniform2fv(uScale, viewScale);
                        gl.uniform2fv(uOffset, viewOffset);
                        gl.uniform4fv(uColor, color);
                        gl.drawArrays(gl.LINE_STRIP, 0, points.length / 2);
                    }
                    points = [];
                };

                for (let i = 0; i < 1440; i++) {
                    let v = telemetryData[offset + i];
                    if (isNaN(v)) flush();
                    else {
                        points.push((i / 1439) * 2 - 1, ((v - min) / (max - min)) * 2 - 1);
                    }
                }
                flush();
            };

            if (currentMode === -1) {
                if (isStacked) {
                    for (let i = 0; i < 5; i++) {
                        drawChannel(i, colors[i], [0.95, 0.18], [0, 0.8 - i * 0.4]);
                    }
                } else {
                    for (let i = 0; i < 5; i++) {
                        drawChannel(i, colors[i], [0.95, 0.9], [0, 0]);
                    }
                }
            } else {
                drawChannel(currentMode, colors[currentMode], [0.95, 0.9], [0, 0]);
            }
        }

        setInterval(fetchData, 2000);
        window.addEventListener('resize', draw);
        fetchData();
    </script>
</body>
</html>
)rawliteral";

void update_minute_tick() {
    if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        buffer_head = (buffer_head + 1) % MINUTES_GRAPH_BUFFER_MAX;

        if (new_packet_received) {
            minutes_buffer[0][buffer_head] = last_frame.radio_active_time;
            minutes_buffer[1][buffer_head] = last_frame.voltage_ADC0;
            minutes_buffer[2][buffer_head] = last_frame.wifi_rssi;
            minutes_buffer[3][buffer_head] = last_frame.SH4x_rel_humidity;
            minutes_buffer[4][buffer_head] = last_frame.SH4x_temperature;
            new_packet_received = false;
        } else {
            for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
                if (minutes_buffer[j]) {
                    minutes_buffer[j][buffer_head] = NAN;
                }
            }
        }
        xSemaphoreGive(bufferMutex);
    }
}

void setup() {
    Serial.begin(115200);

    bufferMutex = xSemaphoreCreateMutex();
    loadSettings();

    // Dynamic allocation of individual buffers
    for (int j = 0; j < NUMBER_OF_BUFFERS; j++) {
        minutes_buffer[j] = (float*)malloc(MINUTES_GRAPH_BUFFER_MAX * sizeof(float));
        if (minutes_buffer[j]) {
            for (int i = 0; i < MINUTES_GRAPH_BUFFER_MAX; i++) {
                minutes_buffer[j][i] = NAN;
            }
        }
    }

    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(ap_ssid, ap_password);
    WiFi.begin(current_sta_ssid.c_str(), current_sta_password.c_str());

    if (udp.listenMulticast(multicastIP, multicastPort)) {
        udp.onPacket([](AsyncUDPPacket packet) {
            if (packet.length() == sizeof(telemetry_frame)) {
                // Mutex protection for last_frame and new_packet_received
                if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    memcpy(&last_frame, packet.data(), sizeof(telemetry_frame));
                    new_packet_received = true;
                    total_packets++;
                    xSemaphoreGive(bufferMutex);
                }
            }
        });
    }

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });

    server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
        // Capture current head under mutex
        int current_head;
        if (xSemaphoreTake(bufferMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            current_head = buffer_head;
            xSemaphoreGive(bufferMutex);
        } else {
            request->send(503, "text/plain", "Busy");
            return;
        }

        // Chunked response to avoid large memory peak and re-order circular buffer
        AsyncWebServerResponse *response = request->beginChunkedResponse("application/octet-stream", [current_head](uint8_t *buffer, size_t maxLen, size_t index) -> size_t {
            size_t total_size = NUMBER_OF_BUFFERS * MINUTES_GRAPH_BUFFER_MAX * sizeof(float);
            if (index >= total_size) return 0;

            size_t global_float_index = index / sizeof(float);
            size_t channel = global_float_index / MINUTES_GRAPH_BUFFER_MAX;
            size_t time_index = global_float_index % MINUTES_GRAPH_BUFFER_MAX;

            // In circular buffer, the oldest data is at (head + 1) % MAX
            // and the newest is at (head) % MAX.
            // We want to send in chronological order: oldest to newest.
            size_t circular_index = (current_head + 1 + time_index) % MINUTES_GRAPH_BUFFER_MAX;

            size_t bytes_to_copy = 0;
            // Send at least 1 float, up to maxLen
            while (bytes_to_copy + sizeof(float) <= maxLen && index + bytes_to_copy < total_size) {
                size_t current_global_float_index = (index + bytes_to_copy) / sizeof(float);
                size_t current_channel = current_global_float_index / MINUTES_GRAPH_BUFFER_MAX;
                size_t current_time_index = current_global_float_index % MINUTES_GRAPH_BUFFER_MAX;
                size_t current_circular_index = (current_head + 1 + current_time_index) % MINUTES_GRAPH_BUFFER_MAX;

                if (minutes_buffer[current_channel]) {
                    memcpy(buffer + bytes_to_copy, &minutes_buffer[current_channel][current_circular_index], sizeof(float));
                } else {
                    float nan_val = NAN;
                    memcpy(buffer + bytes_to_copy, &nan_val, sizeof(float));
                }
                bytes_to_copy += sizeof(float);
            }

            return bytes_to_copy;
        });
        request->send(response);
    });

    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{";
        json += "\"packets\":" + String(total_packets) + ",";
        json += "\"ips\":\"STA: " + WiFi.localIP().toString() + ", AP: " + WiFi.softAPIP().toString() + "\"";
        json += "}";
        request->send(200, "application/json", json);
    });

    server.on("/settings", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("ssid", true) && request->hasParam("pass", true)) {
            String new_ssid = request->getParam("ssid", true)->value();
            String new_pass = request->getParam("pass", true)->value();
            saveSettings(new_ssid, new_pass);
            request->send(200, "text/plain", "Settings saved. Rebooting...");

            // Delayed reboot
            delay(1000);
            ESP.restart();
        } else {
            request->send(400, "text/plain", "Missing parameters");
        }
    });

    server.begin();
}

void loop() {
    static uint32_t last_tick = 0;
    if (millis() - last_tick >= 60000) {
        update_minute_tick();
        last_tick = millis();
    }
}
