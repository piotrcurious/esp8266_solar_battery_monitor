#include "mock_arduino.hpp"
#include <iostream>

// Include the logic from the .ino file but we need to wrap it.
// We'll modify the .ino slightly to be includable or just paste it here.
// Better: create a controller_logic.cpp and include it.

#include "controller_logic.hpp"

int main() {
    controller_setup();
    // Basic SYNC to start the analyzer communication
    std::cout << "SYNC 0" << std::endl;
    while (true) {
        // Wait for analyzer to tell us to continue or exit
        std::string cmd;
        if (!(std::cin >> cmd)) break;
        if (cmd == "EXIT") break;

        controller_loop();

        // The analyzer will control the flow
        // We'll advance by 1ms for the host logic loop (legacy mode uses fixed delays usually)
        _mock_millis += 1;
        std::cout << "SYNC " << _mock_millis << std::endl;
    }
    return 0;
}
