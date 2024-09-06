const int MAX_SIZE = 10; // Define the maximum size of the array
int arr[MAX_SIZE];       // Declare the array
int arrSize = 0;         // Keep track of the number of valid elements in the array

// Function prototypes
void initializeArray();
void addNumber(int number);
void removeNumber(int number);
void printArray();
bool numberExists(int number);

void setup() {
  Serial.begin(9600);
  initializeArray();  // Initialize the array with -1
  printArray();       // Print the initial state of the array

  addNumber(5);
  addNumber(10);
  addNumber(7);
  addNumber(10);      // Attempt to add a number that already exists
  printArray();       // Print after adding some numbers

  removeNumber(10);
  printArray();       // Print after removing a number
}

void loop() {
  // The main loop can be used for additional testing if needed
}

// Function to initialize the array with -1
void initializeArray() {
  for (int i = 0; i < MAX_SIZE; i++) {
    arr[i] = -1;
  }
}

// Function to add a number to the array at the last available position if it doesn't already exist
void addNumber(int number) {
  if (numberExists(number)) {
    Serial.println("Number already exists in the array. No action taken.");
    return;
  }

  if (arrSize < MAX_SIZE) {
    arr[arrSize] = number;
    arrSize++;
  } else {
    Serial.println("Array is full. Cannot add more numbers.");
  }
}

// Function to remove a number from the array and shift elements if necessary
void removeNumber(int number) {
  int index = -1;

  // Find the index of the number to be removed
  for (int i = 0; i < arrSize; i++) {
    if (arr[i] == number) {
      index = i;
      break;
    }
  }

  // If the number was found, remove it and shift the elements
  if (index != -1) {
    for (int i = index; i < arrSize - 1; i++) {
      arr[i] = arr[i + 1];
    }
    arr[arrSize - 1] = -1; // Mark the last position as empty
    arrSize--;
  } else {
    Serial.println("Number not found in the array.");
  }
}

// Function to check if a number exists in the array
bool numberExists(int number) {
  for (int i = 0; i < arrSize; i++) {
    if (arr[i] == number) {
      return true;
    }
  }
  return false;
}

// Function to print the current state of the array
void printArray() {
  Serial.print("Array: ");
  for (int i = 0; i < MAX_SIZE; i++) {
    Serial.print(arr[i]);
    Serial.print(" ");
  }
  Serial.println();
}
