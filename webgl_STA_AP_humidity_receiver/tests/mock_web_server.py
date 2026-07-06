import http.server
import socketserver
import struct
import json
import math
import sys
import cgi

PORT = 8002 # Changed port again
MINUTES_GRAPH_BUFFER_MAX = 1440
NUMBER_OF_BUFFERS = 5

mock_data = []
for j in range(NUMBER_OF_BUFFERS):
    buffer = []
    for i in range(MINUTES_GRAPH_BUFFER_MAX):
        val = math.sin(i / 100.0 + j) * 10 + 20
        buffer.append(val)
    mock_data.append(buffer)

class MockESP32Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('webgl_STA_AP_humidity_receiver/webgl_STA_AP_humidity_receiver.ino', 'r') as f:
                content = f.read()
                start = content.find('R"rawliteral(') + 12
                end = content.find(')rawliteral"')
                self.wfile.write(content[start:end].encode())
        elif self.path == '/data':
            self.send_response(200)
            self.send_header('Content-type', 'application/octet-stream')
            self.end_headers()
            flat_data = [item for sublist in mock_data for item in sublist]
            binary_data = struct.pack('%sf' % len(flat_data), *flat_data)
            self.wfile.write(binary_data)
        elif self.path == '/status':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            status = {"packets": 1234, "ips": "STA: 192.168.1.100, AP: 192.168.4.1"}
            self.wfile.write(json.dumps(status).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/settings':
            form = cgi.FieldStorage(
                fp=self.rfile,
                headers=self.headers,
                environ={'REQUEST_METHOD': 'POST'}
            )
            ssid = form.getvalue('ssid')
            passw = form.getvalue('pass')
            print(f"Received Settings: SSID={ssid}, PASS={passw}")
            self.send_response(200)
            self.send_header('Content-type', 'text/plain')
            self.end_headers()
            self.wfile.write(b"Settings saved. Rebooting...")
        else:
            self.send_error(404)

socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", PORT), MockESP32Handler) as httpd:
    print(f"Serving mock ESP32 at http://localhost:{PORT}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
