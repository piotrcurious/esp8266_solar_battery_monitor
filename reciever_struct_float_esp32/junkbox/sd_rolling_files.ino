#include <SPI.h>
#include <SD.h>

const int chipSelect = 5;  // Change this to the appropriate pin for your board
const int maxFiles = 5;    // Maximum number of files to store

void setup() {
  Serial.begin(115200);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");
}

void loop() {
  // Dummy data to store
  int dataArray[] = {1, 2, 3, 4, 5};
  int dataSize = sizeof(dataArray) / sizeof(dataArray[0]);

  storeData(dataArray, dataSize);

  delay(10000); // Delay for demonstration purposes
}

void storeData(int *dataArray, int dataSize) {
  // First, find the latest file index
  int fileIndex = findLatestFileIndex();

  // Check if the number of files exceeds the maximum allowed
  if (fileIndex >= maxFiles - 1) {
    // Remove the oldest file
    SD.remove(fileName(0));

    // Rename all other files in a rolling fashion
    for (int i = 1; i <= fileIndex; i++) {
      SD.rename(fileName(i), fileName(i - 1));
    }

    fileIndex = maxFiles - 2;
  }

  // Increment the file index for the new file
  fileIndex++;

  // Create and write data to the new file
  File dataFile = SD.open(fileName(fileIndex), FILE_WRITE);
  if (dataFile) {
    for (int i = 0; i < dataSize; i++) {
      dataFile.println(dataArray[i]);
    }
    dataFile.close();
    Serial.println("Data stored successfully.");
  } else {
    Serial.println("Failed to open file for writing.");
  }
}

String fileName(int index) {
  // Generate a file name in the format "0x.hex", where x is a hexadecimal number
  char fileName[10];
  snprintf(fileName, sizeof(fileName), "%01X.txt", index);
  return String(fileName);
}

int findLatestFileIndex() {
  int index = -1;
  for (int i = 0; i < maxFiles; i++) {
    if (SD.exists(fileName(i))) {
      index = i;
    } else {
      break;
    }
  }
  return index;
}
