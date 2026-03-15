#include <iostream>
#include <string>
#include <sstream>
#include "BatteryTester.hpp"

int main() {
    BatteryTester tester;
    tester.startTest();

    std::string line;
    // Format: time_ms,measured_voltage,measured_current
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        unsigned long time_ms;
        float v, i;
        char comma;

        if (ss >> time_ms >> comma >> v >> comma >> i) {
            tester.update(time_ms, v, i);

            // Output state for emulator/analyzer
            std::cout << time_ms << ","
                      << tester.getPwmOutput() << ","
                      << tester.getEstimatedVoltage() << ","
                      << tester.getEstimatedSlope() << ","
                      << tester.getState() << ","
                      << tester.getTargetCurrent() << std::endl;
        }
    }
    return 0;
}
