// Compressed rolling buffer with exponential delta compression
// Stores timestamp/value pairs as 8-bit deltas relative to newest value

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
    
    // Compression scale factors
    const float TIME_SCALE = 0.1f;    // 100ms resolution
    const float VALUE_SCALE = 0.01f;  // 1% resolution
    
    // Get previous index in circular buffer
    int getPrevIndex(int index) {
        return (index - 2 + BUFFER_SIZE) % BUFFER_SIZE;
    }
    
    // Get next index in circular buffer
    int getNextIndex(int index) {
        return (index + 2) % BUFFER_SIZE;
    }
    
    // Convert float delta to compressed 8-bit representation
    uint8_t compressDelta(float delta, float scale) {
        if (delta == 0) return 0;
        
        float absDelta = abs(delta);
        float scaled = absDelta / scale;
        float compressed = log2f(1 + scaled);
        
        // Map to 0-127 range and set sign bit
        uint8_t result = min(127, (uint8_t)(compressed * 32));
        if (delta < 0) {
            result |= 0x80;  // Set sign bit
        }
        return result;
    }
    
    // Convert compressed 8-bit value back to float delta
    float expandDelta(uint8_t compressed, float scale) {
        if (compressed == 0) return 0;
        
        // Extract magnitude and sign
        bool negative = (compressed & 0x80) != 0;
        float magnitude = (compressed & 0x7F) / 32.0f;
        
        // Reverse exponential compression
        float expanded = (pow(2, magnitude) - 1) * scale;
        return negative ? -expanded : expanded;
    }
    
    // Recompress the entire buffer relative to new current values
    void recompressBuffer() {
        if (count <= 1) return;
        
        // Temporary storage for decompressed values
        struct Entry {
            float timestamp;
            float value;
        };
        Entry* entries = new Entry[count];
        
        // Decompress all entries
        int readIdx = head;
        int entryIdx = 0;
        
        // First entry is current values
        entries[entryIdx++] = {currentTimestamp, currentValue};
        
        // Decompress remaining entries
        for (int i = 1; i < count; i++) {
            readIdx = getPrevIndex(readIdx);
            float timeDelta = expandDelta(buffer[readIdx], TIME_SCALE);
            float valueDelta = expandDelta(buffer[readIdx + 1], VALUE_SCALE);
            
            entries[entryIdx].timestamp = entries[entryIdx - 1].timestamp + timeDelta;
            entries[entryIdx].value = entries[entryIdx - 1].value + valueDelta;
            entryIdx++;
        }
        
        // Recompress relative to current values
        int writeIdx = head;
        for (int i = 1; i < count; i++) {
            writeIdx = getPrevIndex(writeIdx);
            float timeDelta = entries[i].timestamp - currentTimestamp;
            float valueDelta = entries[i].value - currentValue;
            
            buffer[writeIdx] = compressDelta(timeDelta, TIME_SCALE);
            buffer[writeIdx + 1] = compressDelta(valueDelta, VALUE_SCALE);
        }
        
        delete[] entries;
    }

public:
    // Add a new timestamp/value pair to the buffer
    bool push(float timestamp, float value) {
        if (count >= BUFFER_SIZE/2) {
            // Buffer full, remove oldest entry
            tail = getNextIndex(tail);
            count--;
        }
        
        // Store new value at head
        head = getPrevIndex(head);
        
        // If this is the first entry
        if (count == 0) {
            currentTimestamp = timestamp;
            currentValue = value;
            buffer[head] = 0;     // Zero delta for first entry
            buffer[head + 1] = 0;
            count++;
            return true;
        }
        
        // Calculate deltas for existing entries relative to new value
        float timeDelta = currentTimestamp - timestamp;
        float valueDelta = currentValue - value;
        
        // Store compressed deltas of old current relative to new current
        buffer[head] = compressDelta(timeDelta, TIME_SCALE);
        buffer[head + 1] = compressDelta(valueDelta, VALUE_SCALE);
        
        // Update current values
        currentTimestamp = timestamp;
        currentValue = value;
        
        count++;
        
        // Periodically recompress buffer to maintain accuracy
        if (count % 100 == 0) {
            recompressBuffer();
        }
        
        return true;
    }
    
    // Read oldest timestamp/value pair from buffer
    bool pop(float& timestamp, float& value) {
        if (count == 0) return false;
        
        if (tail == head) {
            // Reading the newest (current) value
            timestamp = currentTimestamp;
            value = currentValue;
        } else {
            // Read and expand deltas from oldest entry
            uint8_t compressedTime = buffer[tail];
            uint8_t compressedValue = buffer[tail + 1];
            
            float timeDelta = expandDelta(compressedTime, TIME_SCALE);
            float valueDelta = expandDelta(compressedValue, VALUE_SCALE);
            
            // Calculate actual values relative to current
            timestamp = currentTimestamp + timeDelta;
            value = currentValue + valueDelta;
        }
        
        // Move tail forward
        tail = getNextIndex(tail);
        count--;
        
        return true;
    }
    
    // Peek at oldest value without removing it
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
    
    // Get number of timestamp/value pairs in buffer
    int available() {
        return count;
    }
    
    // Clear the buffer
    void clear() {
        head = 0;
        tail = 0;
        count = 0;
        currentTimestamp = 0;
        currentValue = 0;
    }
};

// Example usage
void setup() {
    Serial.begin(115200);
    CompressedBuffer buffer;
    
    // Add some test data
    buffer.push(0.1, 10.0);
    buffer.push(0.2, 10.5);
    buffer.push(0.3, 9.8);
    buffer.push(0.4, 10.2);
    
    // Read back compressed data
    float timestamp, value;
    while (buffer.pop(timestamp, value)) {
        Serial.printf("Time: %.3f, Value: %.3f\n", timestamp, value);
    }
}

void loop() {
    // Main program loop
}
