#include "FS.h"
#include "SPIFFS.h"

void setup() {
  Serial.begin(115200);

  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
}
void writeArrayToFile(const char* path, int* array, size_t size) {
  File file = SPIFFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  for (size_t i = 0; i < size; i++) {
    file.write((uint8_t*)&array[i], sizeof(int));
  }

  file.close();
  Serial.println("Array written to file");
}

void readArrayFromFile(const char* path, int* array, size_t size) {
  File file = SPIFFS.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  for (size_t i = 0; i < size; i++) {
    if (file.available()) {
      file.read((uint8_t*)&array[i], sizeof(int));
    }
  }

  file.close();
  Serial.println("Array read from file");
}
void loop() {
  const char* path = "/array.bin";
  
  // Create an example array
  int dataToWrite[5] = {10, 20, 30, 40, 50};
  
  // Write the array to SPIFFS
  writeArrayToFile(path, dataToWrite, 5);

  // Create an empty array to read data into
  int dataRead[5] = {0};

  // Read the array from SPIFFS
  readArrayFromFile(path, dataRead, 5);

  // Print out the read array
  Serial.println("Array contents:");
  for (size_t i = 0; i < 5; i++) {
    Serial.println(dataRead[i]);
  }

  delay(10000); // Delay for 10 seconds before repeating
}

