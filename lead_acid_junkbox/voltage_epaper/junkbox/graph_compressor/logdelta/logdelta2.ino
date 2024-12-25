#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Improved compressed rolling buffer with display capabilities
class CompressedBuffer {
private:
    static const int BUFFER_SIZE = 1024;  // Size in bytes for timestamps/values
    uint8_t buffer[BUFFER_SIZE];
    
    // Circular buffer tracking
    int head = 0;  // Points to newest element
    int tail = 0;  // Points to oldest element
    int count = 0;
    
    // Current actual values (newest in buffer)
    float currentTimestamp = 0;
    float currentValue = 0;
    
    // Track min/max values for auto-scaling
    float minValue = INFINITY;
    float maxValue = -INFINITY;
    float timeWindow = 60.0f;  // Default 60-second window
    
    // Compression scale factors
    const float TIME_SCALE = 0.1f;    // 100ms resolution
    const float VALUE_SCALE = 0.01f;  // 1% resolution
    
    // Compression helper functions (same as before)
    int getPrevIndex(int index) { return (index - 2 + BUFFER_SIZE) % BUFFER_SIZE; }
    int getNextIndex(int index) { return (index + 2) % BUFFER_SIZE; }
    
    uint8_t compressDelta(float delta, float scale) {
        if (delta == 0) return 0;
        float absDelta = abs(delta);
        float scaled = absDelta / scale;
        float compressed = log2f(1 + scaled);
        uint8_t result = min(127, (uint8_t)(compressed * 32));
        if (delta < 0) result |= 0x80;
        return result;
    }
    
    float expandDelta(uint8_t compressed, float scale) {
        if (compressed == 0) return 0;
        bool negative = (compressed & 0x80) != 0;
        float magnitude = (compressed & 0x7F) / 32.0f;
        float expanded = (pow(2, magnitude) - 1) * scale;
        return negative ? -expanded : expanded;
    }
    
    void updateMinMax(float value) {
        minValue = min(minValue, value);
        maxValue = max(maxValue, value);
    }
    
    void recompressBuffer() {
        if (count <= 1) return;
        
        std::vector<std::pair<float, float>> entries;
        entries.reserve(count);
        
        // Decompress all entries
        int readIdx = head;
        entries.push_back({currentTimestamp, currentValue});
        
        for (int i = 1; i < count; i++) {
            readIdx = getPrevIndex(readIdx);
            float timeDelta = expandDelta(buffer[readIdx], TIME_SCALE);
            float valueDelta = expandDelta(buffer[readIdx + 1], VALUE_SCALE);
            
            float timestamp = entries.back().first + timeDelta;
            float value = entries.back().second + valueDelta;
            entries.push_back({timestamp, value});
        }
        
        // Recompress relative to current values
        int writeIdx = head;
        for (int i = 1; i < count; i++) {
            writeIdx = getPrevIndex(writeIdx);
            float timeDelta = entries[i].first - currentTimestamp;
            float valueDelta = entries[i].second - currentValue;
            
            buffer[writeIdx] = compressDelta(timeDelta, TIME_SCALE);
            buffer[writeIdx + 1] = compressDelta(valueDelta, VALUE_SCALE);
        }
    }

public:
    CompressedBuffer(float timeWindowSeconds = 60.0f) : timeWindow(timeWindowSeconds) {}
    
    bool push(float timestamp, float value) {
        if (count >= BUFFER_SIZE/2) {
            tail = getNextIndex(tail);
            count--;
        }
        
        head = getPrevIndex(head);
        
        if (count == 0) {
            currentTimestamp = timestamp;
            currentValue = value;
            buffer[head] = 0;
            buffer[head + 1] = 0;
            updateMinMax(value);
            count++;
            return true;
        }
        
        float timeDelta = currentTimestamp - timestamp;
        float valueDelta = currentValue - value;
        
        buffer[head] = compressDelta(timeDelta, TIME_SCALE);
        buffer[head + 1] = compressDelta(valueDelta, VALUE_SCALE);
        
        currentTimestamp = timestamp;
        currentValue = value;
        updateMinMax(value);
        
        count++;
        
        if (count % 100 == 0) {
            recompressBuffer();
        }
        
        // Remove old data outside time window
        while (count > 1) {
            float oldestTime;
            float oldestValue;
            if (!peek(oldestTime, oldestValue)) break;
            
            if (currentTimestamp - oldestTime > timeWindow) {
                tail = getNextIndex(tail);
                count--;
            } else {
                break;
            }
        }
        
        return true;
    }
    
    bool pop(float& timestamp, float& value) {
        if (count == 0) return false;
        
        if (tail == head) {
            timestamp = currentTimestamp;
            value = currentValue;
        } else {
            uint8_t compressedTime = buffer[tail];
            uint8_t compressedValue = buffer[tail + 1];
            
            float timeDelta = expandDelta(compressedTime, TIME_SCALE);
            float valueDelta = expandDelta(compressedValue, VALUE_SCALE);
            
            timestamp = currentTimestamp + timeDelta;
            value = currentValue + valueDelta;
        }
        
        tail = getNextIndex(tail);
        count--;
        return true;
    }
    
    bool peek(float& timestamp, float& value) {
        if (count == 0) return false;
        
        if (tail == head) {
            timestamp = currentTimestamp;
            value = currentValue;
        } else {
            uint8_t compressedTime = buffer[tail];
            uint8_t compressedValue = buffer[tail + 1];
            
            float timeDelta = expandDelta(compressedTime, TIME_SCALE);
            float valueDelta = expandDelta(compressedValue, VALUE_SCALE);
            
            timestamp = currentTimestamp + timeDelta;
            value = currentValue + valueDelta;
        }
        return true;
    }
    
    void getValueRange(float& min, float& max) {
        min = minValue;
        max = maxValue;
    }
    
    float getCurrentValue() { return currentValue; }
    float getCurrentTimestamp() { return currentTimestamp; }
    int available() { return count; }
    
    void clear() {
        head = 0;
        tail = 0;
        count = 0;
        currentTimestamp = 0;
        currentValue = 0;
        minValue = INFINITY;
        maxValue = -INFINITY;
    }
    
    // Draw the buffer contents on the OLED display
    void drawGraph(Adafruit_SSD1306& display) {
        if (count == 0) return;
        
        const int GRAPH_HEIGHT = 48;
        const int GRAPH_OFFSET_Y = 56;
        const int TEXT_OFFSET_Y = 8;
        
        display.clearDisplay();
        
        // Draw current value as text
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print(currentValue, 1);
        
        // Calculate value range for scaling
        float valueRange = maxValue - minValue;
        if (valueRange < 0.1f) valueRange = 0.1f;  // Prevent division by zero
        
        // Store all points for drawing
        std::vector<std::pair<float, float>> points;
        points.reserve(count);
        
        // Add current point
        points.push_back({currentTimestamp, currentValue});
        
        // Decompress and add all points
        int readIdx = head;
        for (int i = 1; i < count; i++) {
            readIdx = getPrevIndex(readIdx);
            float timeDelta = expandDelta(buffer[readIdx], TIME_SCALE);
            float valueDelta = expandDelta(buffer[readIdx + 1], VALUE_SCALE);
            
            float timestamp = points.back().first + timeDelta;
            float value = points.back().second + valueDelta;
            points.push_back({timestamp, value});
        }
        
        // Draw the graph
        int lastX = -1;
        int lastY = -1;
        
        for (const auto& point : points) {
            float timeOffset = currentTimestamp - point.first;
            int x = SCREEN_WIDTH - (timeOffset / timeWindow) * SCREEN_WIDTH;
            
            if (x < 0) continue;
            if (x >= SCREEN_WIDTH) continue;
            
            float normalizedValue = (point.second - minValue) / valueRange;
            int y = GRAPH_OFFSET_Y - normalizedValue * GRAPH_HEIGHT;
            
            if (lastX >= 0) {
                display.drawLine(lastX, lastY, x, y, SSD1306_WHITE);
            }
            
            lastX = x;
            lastY = y;
        }
        
        // Draw axes
        display.drawLine(0, GRAPH_OFFSET_Y, SCREEN_WIDTH-1, GRAPH_OFFSET_Y, SSD1306_WHITE);
        display.drawLine(SCREEN_WIDTH-1, TEXT_OFFSET_Y, SCREEN_WIDTH-1, GRAPH_OFFSET_Y, SSD1306_WHITE);
        
        display.display();
    }
};

// Test data generator
class TestDataGenerator {
private:
    float time = 0;
    const float frequency = 0.1f;  // Hz
    const float amplitude = 5.0f;
    const float noise = 0.5f;
    
public:
    std::pair<float, float> getNext() {
        float value = amplitude * sin(2 * PI * frequency * time) + 
                     random(-noise, noise) +  // Add noise
                     2 * sin(2 * PI * 0.05f * time);  // Add slower variation
        
        time += 0.1f;  // 100ms intervals
        return {time, value};
    }
};

// Global instances
CompressedBuffer buffer(30.0f);  // 30-second window
TestDataGenerator testGen;

void setup() {
    Serial.begin(115200);
    
    // Initialize OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;);
    }
    display.clearDisplay();
    display.setRotation(0);
    display.display();
    
    // Initial display setup
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
}

void loop() {
    static unsigned long lastUpdate = 0;
    const unsigned long UPDATE_INTERVAL = 100;  // 100ms update rate
    
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
        lastUpdate = currentMillis;
        
        // Get next test data point
        auto [timestamp, value] = testGen.getNext();
        
        // Add to buffer
        buffer.push(timestamp, value);
        
        // Update display
        buffer.drawGraph(display);
        
        // Print to Serial for debugging
        Serial.printf("Time: %.1f, Value: %.2f\n", timestamp, value);
    }
}
