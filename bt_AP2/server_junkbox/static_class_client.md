Certainly! We can refactor the code to use classes with static methods to create instances, similar to how the server code was organized. This approach keeps the client code organized while providing the simplicity of using functions.

### Updated Client Code with Classes and Static Constructors

```cpp
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";
const char* serverIP = "ESP32_SERVER_IP";  // Replace with your ESP32 server's IP

// Base class for managing HTTP clients
class HttpClient {
protected:
    const char* serverIP;
    const char* path;
    HTTPClient http;

    HttpClient(const char* ip, const char* p) : serverIP(ip), path(p) {}

public:
    virtual void retrieve() = 0;
};

// Class for handling float variable retrieval
class FloatClient : public HttpClient {
private:
    float value;

    // Private constructor to be used by static method
    FloatClient(const char* ip, const char* p) : HttpClient(ip, p), value(0.0) {}

public:
    static FloatClient create(const char* ip, const char* p) {
        return FloatClient(ip, p);
    }

    void retrieve() override {
        String url = String("http://") + serverIP + path;
        http.begin(url);
        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            http.getStream().readBytes((uint8_t*)&value, sizeof(float));
            Serial.printf("Retrieved float from %s: %f\n", path, value);
        } else {
            Serial.printf("Failed to retrieve float from %s\n", path);
        }
        http.end();
    }

    float getValue() const { return value; }
};

// Class for handling float array retrieval
class FloatArrayClient : public HttpClient {
private:
    float* array;
    size_t size;

    // Private constructor to be used by static method
    FloatArrayClient(const char* ip, const char* p, float* arr, size_t s)
        : HttpClient(ip, p), array(arr), size(s) {}

public:
    static FloatArrayClient create(const char* ip, const char* p, float* arr, size_t s) {
        return FloatArrayClient(ip, p, arr, s);
    }

    void retrieve() override {
        String url = String("http://") + serverIP + path;
        http.begin(url);
        int httpCode = http.GET();

        if (httpCode == HTTP_CODE_OK) {
            http.getStream().readBytes((uint8_t*)array, sizeof(float) * size);
            Serial.printf("Retrieved float array from %s:\n", path);
            for (size_t i = 0; i < size; i++) {
                Serial.printf("  [%d] = %f\n", i, array[i]);
            }
        } else {
            Serial.printf("Failed to retrieve float array from %s\n", path);
        }
        http.end();
    }

    const float* getArray() const { return array; }
    size_t getSize() const { return size; }
};

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Create clients using static constructors
    FloatClient floatClient = FloatClient::create(serverIP, "/float");
    floatClient.retrieve();
    float retrievedFloat = floatClient.getValue();
    Serial.printf("Retrieved float value: %f\n", retrievedFloat);

    float retrievedArray[5];
    FloatArrayClient arrayClient = FloatArrayClient::create(serverIP, "/array", retrievedArray, 5);
    arrayClient.retrieve();
    const float* array = arrayClient.getArray();
    Serial.println("Retrieved array values:");
    for (size_t i = 0; i < arrayClient.getSize(); i++) {
        Serial.printf("  [%d] = %f\n", i, array[i]);
    }
}

void loop() {
    // Example usage: periodic retrieval
    delay(5000);

    // Reuse the same client instances to retrieve data again
    // FloatClient and FloatArrayClient could be used as in setup()
}
```

### Explanation of the Refactored Code

#### 1. **Classes with Static Constructors**

- **`FloatClient`** and **`FloatArrayClient`**: These classes are responsible for retrieving a single float value and an array of floats, respectively.
- Each class has a private constructor and a static method called `create` that acts as a factory to instantiate the class objects.

  ```cpp
  static FloatClient create(const char* ip, const char* p) {
      return FloatClient(ip, p);
  }
  ```

  This mirrors the server-side code pattern where static methods or functions are used to create instances.

#### 2. **Class Structure**

- **`HttpClient`**: This is a base class that holds common attributes like `serverIP` and `path` and defines a pure virtual function `retrieve()`, which derived classes implement.

- **`FloatClient`**: Inherits from `HttpClient`, manages retrieval of a single float value, and stores the result in the `value` member.

- **`FloatArrayClient`**: Inherits from `HttpClient`, manages retrieval of an array of floats, and stores the result in a pointer `array`.

#### 3. **Path Handling**

- The path is passed to the static `create` method when an instance is created, and it's stored as a member variable in the `HttpClient` base class.

  ```cpp
  FloatClient floatClient = FloatClient::create(serverIP, "/float");
  ```

  Here, `"/float"` is the path that will be used in the HTTP request to the server.

#### 4. **Retrieving Data**

- **`retrieve()`**: This method constructs the URL using the `serverIP` and `path`, sends an HTTP GET request, and then reads the data into either a single float or an array.

  ```cpp
  void retrieve() override {
      String url = String("http://") + serverIP + path;
      http.begin(url);
      int httpCode = http.GET();
      // ...
  }
  ```

#### 5. **Usage in `setup()` and `loop()`**

- The `setup()` function demonstrates creating instances of `FloatClient` and `FloatArrayClient` using the static constructors. It then retrieves and prints the values.

- The `loop()` function is left as a placeholder where you can perform periodic retrievals using the same client instances.

### Benefits of This Approach

- **Encapsulation**: The logic for retrieving and handling different types of data is encapsulated within each class, making the code easier to maintain and extend.
- **Consistency**: The use of static constructors parallels the server-side code, providing a consistent pattern across client and server.
- **Flexibility**: New data types or retrieval methods can be easily added by creating new classes with similar static constructor patterns.

This structure balances simplicity with organization, making it easy to manage and extend the client code as your application grows.
