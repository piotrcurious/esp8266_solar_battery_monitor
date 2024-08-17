To improve the refactoring by creating a `globals.h` file that acts as a relay for variables and constants that need to be global, we can follow a structured approach where:

1. **Global variables** are declared in `globals.h`.
2. **Global constants** are defined in separate, function-specific configuration files.
3. The configuration files initialize or configure these global variables.
4. `globals.h` only includes declarations of the variables and includes the necessary configuration files.

### Step 1: Define the Global Variables in `globals.h`

This file will contain external declarations of the variables, which means it tells the compiler that these variables exist and will be defined somewhere else. This way, any part of the code that includes `globals.h` can access these variables.

```cpp
// globals.h

#ifndef GLOBALS_H
#define GLOBALS_H

// Declare global variables
extern int globalVariable1;
extern float globalVariable2;
extern const int GLOBAL_CONSTANT1;
extern const float GLOBAL_CONSTANT2;

// Include the configuration headers where these are defined
#include "config_a.h"
#include "config_b.h"

#endif // GLOBALS_H
```

### Step 2: Create Function-Related Configuration Files

These files are responsible for defining and initializing the global variables. By keeping the configurations modular, it becomes easier to manage and update the global values.

#### Example: `config_a.h`

```cpp
// config_a.h

#ifndef CONFIG_A_H
#define CONFIG_A_H

// Define and initialize the global variables for Function A
int globalVariable1 = 10; // Example: Value set based on configuration A
const int GLOBAL_CONSTANT1 = 42; // Example: A constant used in Function A

#endif // CONFIG_A_H
```

#### Example: `config_b.h`

```cpp
// config_b.h

#ifndef CONFIG_B_H
#define CONFIG_B_H

// Define and initialize the global variables for Function B
float globalVariable2 = 3.14f; // Example: Value set based on configuration B
const float GLOBAL_CONSTANT2 = 2.718f; // Example: A constant used in Function B

#endif // CONFIG_B_H
```

### Step 3: Use Global Variables in Source Files

In any source file where these global variables and constants are required, simply include `globals.h`.

```cpp
// main.cpp

#include "globals.h"
#include <iostream>

void functionA() {
    std::cout << "Global Variable 1: " << globalVariable1 << std::endl;
    std::cout << "Global Constant 1: " << GLOBAL_CONSTANT1 << std::endl;
}

void functionB() {
    std::cout << "Global Variable 2: " << globalVariable2 << std::endl;
    std::cout << "Global Constant 2: " << GLOBAL_CONSTANT2 << std::endl;
}

int main() {
    functionA();
    functionB();
    return 0;
}
```

### Explanation and Benefits

1. **Modularity**: The global variables and constants are defined in separate configuration files (`config_a.h`, `config_b.h`). This makes the code modular and easier to maintain. Changes to global variables or constants can be made in the respective configuration file without affecting the entire project.

2. **Separation of Concerns**: By having function-related configuration files, each part of the project is only concerned with the variables and constants it actually uses. This reduces the risk of conflicts or unintended side effects.

3. **Globals Relay**: `globals.h` acts as a relay by including the necessary configuration headers and declaring the global variables. It ensures that any source file including `globals.h` will have access to all necessary global variables and constants without needing to directly include multiple configuration files.

This refactoring approach increases the maintainability, readability, and scalability of the codebase by ensuring that global variables are well-organized and their initialization is centralized in a logical manner.
