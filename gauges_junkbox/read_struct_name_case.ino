// Define a struct type called CarSpeed with four int elements
struct CarSpeed {
  int fr; // front right speed
  int fl; // front left speed
  int br; // back right speed
  int bl; // back left speed
};

// Declare a function that takes a pointer to a CarSpeed struct and a string as parameters, and returns a float
float readStructElement(CarSpeed* cs, String name) {
  // Use a switch-case statement to compare the name with the possible struct element names
  switch (name) {
    case "fr": // if the name is "fr", return the fr element as a float
      return (float) cs->fr;
      break;
    case "fl": // if the name is "fl", return the fl element as a float
      return (float) cs->fl;
      break;
    case "br": // if the name is "br", return the br element as a float
      return (float) cs->br;
      break;
    case "bl": // if the name is "bl", return the bl element as a float
      return (float) cs->bl;
      break;
    default: // if the name is not a valid struct element name, return 0.0 as a default value
      return 0.0;
      break;
  }
}

// Example usage of the function
void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  // Create a CarSpeed struct variable and assign some values to its elements
  CarSpeed cs;
  cs.fr = 10;
  cs.fl = 20;
  cs.br = 30;
  cs.bl = 40;
  // Call the function with different names and print the results
  Serial.println(readStructElement(&cs, "fr")); // prints 10.00
  Serial.println(readStructElement(&cs, "fl")); // prints 20.00
  Serial.println(readStructElement(&cs, "br")); // prints 30.00
  Serial.println(readStructElement(&cs, "bl")); // prints 40.00
  Serial.println(readStructElement(&cs, "foo")); // prints 0.00
}

void loop() {
  // Do nothing
}
