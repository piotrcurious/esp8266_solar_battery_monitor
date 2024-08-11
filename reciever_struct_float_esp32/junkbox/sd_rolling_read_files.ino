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

  // Example usage of readData function
  int dataArray[10];
  int dataSize = sizeof(dataArray) / sizeof(dataArray[0]);

  int result = readData(0, dataArray, dataSize);
  if (result != -1) {
    Serial.println("Data read successfully:");
    for (int i = 0; i < result; i++) {
      Serial.println(dataArray[i]);
    }
  } else {
    Serial.println("File does not exist.");
  }
}

void loop() {
  // The main loop can be used to periodically read or store data
}

int readData(int fileIndex, int *dataArray, int maxSize) {
  // Get the current number of files on the SD card
  int latestFileIndex = findLatestFileIndex();

  // Determine the actual file index to read from
  int actualFileIndex = latestFileIndex - fileIndex;

  if (actualFileIndex < 0 || actualFileIndex > latestFileIndex) {
    // File does not exist
    return -1;
  }

  // Open the file
  File dataFile = SD.open(fileName(actualFileIndex), FILE_READ);
  if (!dataFile) {
    // File does not exist or cannot be opened
    return -1;
  }

  // Read data into the array
  int i = 0;
  while (dataFile.available() && i < maxSize) {
    dataArray[i] = dataFile.parseInt();
    i++;
  }
  dataFile.close();

  return i;  // Return the number of elements read
}

String fileName(int index) {
  // Generate a file name in the format "0x.txt", where x is a hexadecimal number
  char fileName[10];
  snprintf(fileName, sizeof(fileName), "%01X.txt", index);
  return String(fileName);
}

int findLatestFileIndex() {
  int left = 0;
  int right = maxFiles - 1;

  while (left <= right) {
    int mid = (left + right) / 2;

    if (SD.exists(fileName(mid))) {
      // File exists, so the latest file must be at mid or to the right
      left = mid + 1;
    } else {
      // File doesn't exist, so the latest file must be to the left
      right = mid - 1;
    }
  }

  // After the loop, right should point to the latest existing file
  return right;
}
