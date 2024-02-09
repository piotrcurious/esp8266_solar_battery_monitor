// Define a struct type called CarSpeed with four int elements
struct CarSpeed {
  int fr; // front right speed
  int fl; // front left speed
  int br; // back right speed
  int bl; // back left speed
};

// Declare a macro that defines a function with the given name and struct element
#define READ_STRUCT_ELEMENT(name, element) \
float name(CarSpeed* cs) { \
  return (float) cs->element; \
}

// Use the macro to define four functions that read each struct element
READ_STRUCT_ELEMENT(readFR, fr)
READ_STRUCT_ELEMENT(readFL, fl)
READ_STRUCT_ELEMENT(readBR, br)
READ_STRUCT_ELEMENT(readBL, bl)

// Example usage of the functions
void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  // Create a CarSpeed struct variable and assign some values to its elements
  CarSpeed cs;
  cs.fr = 10;
  cs.fl = 20;
  cs.br = 30;
  cs.bl = 40;
  // Call the functions with the struct pointer and print the results
  Serial.println(readFR(&cs)); // prints 10.00
  Serial.println(readFL(&cs)); // prints 20.00
  Serial.println(readBR(&cs)); // prints 30.00
  Serial.println(readBL(&cs)); // prints 40.00
}

void loop() {
  // Do nothing
}
