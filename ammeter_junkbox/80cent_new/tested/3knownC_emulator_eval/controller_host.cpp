#include "mock_arduino.hpp"
#include <iostream>

// Include the logic from the .ino file but we need to wrap it.
// We'll modify the .ino slightly to be includable or just paste it here.
// Better: create a controller_logic.cpp and include it.

#include "controller_logic.hpp"

int main() {
    controller_setup();
    while (true) {
        std::cout << "TICK" << std::endl;
        controller_loop();
        // The analyzer will control the flow
        // We'll send a SYNC signal
        std::cout << "SYNC " << _mock_millis << std::endl;

        // Wait for analyzer to tell us to continue or exit
        std::string cmd;
        if (!(std::cin >> cmd)) break;
        if (cmd == "EXIT") break;
        // if cmd is TICK, we continue
        unsigned long next_millis;
        std::cin >> next_millis;
        _mock_millis = next_millis;
    }
    return 0;
}
