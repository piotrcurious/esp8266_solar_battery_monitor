#include "mock_arduino.hpp"
#include "logic_oscillator_feedback.cpp"
#include <stdio.h>
#include <iostream>
#include <string>

RC_Oscillator_Controller controller;

void controller_setup() {
    // Already initialized by constructor
}

void controller_loop() {
    float v = (float)analogRead(A0) * 25.0f / 1024.0f;
    float i_src = (float)analogRead(A1) * 10.0f / 1024.0f;

    controller.update(v, i_src);

    analogWrite(9, (int)(controller.getPwmDuty() * 255));

    // Telemetry for analyzer.py
    // Format: DATA:time_ms:v:i_src:pwm_duty:voc_est:r_est:load_est
    static unsigned long last_telemetry = 0;
    if (millis() - last_telemetry > 10) {
        printf("DATA:%lu:%.3f:%.3f:%.3f:%.3f:%.3f:%.3f\n",
               millis(), v, i_src, controller.getPwmDuty(),
               controller.getVocEst(), controller.getRintEst(), controller.getLoadEst());
        last_telemetry = millis();
    }
}

int main() {
    // Basic SYNC to start the analyzer communication
    printf("SYNC 0\n");
    fflush(stdout);
    while(true) {
        // Wait for TICK from analyzer to prevent deadlock/race
        std::string cmd;
        if (!(std::cin >> cmd)) {
            break;
        }

        controller_loop();

        // Advance time in the mock system
        delay(1);
        printf("SYNC %lu\n", millis());
        fflush(stdout);
    }
    return 0;
}
