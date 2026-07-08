#include "arduino_mock.h"
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstring>
#include <signal.h>
#include <vector>
#include <string>

float readVinPanel();
#include "esp32_solar_battery_controller-1.cpp"

extern int _mock_duty_ch;
float mock_v_pnl = 18.0;
float readVinPanel() { return mock_v_pnl; }

bool run_test(const std::string& scenario) {
    printf("\n--- STARTING TEST SCENARIO: %s ---\n", scenario.c_str());
    _mock_millis = 0;
    _mock_duty_ch = 0;
    _mock_serial_buf.clear();
    mode = MODE_BOOT;
    state = PersistedState{};
    C_baseline = -1;
    internalResistance = 0.05f;
    previousMillis = 0; // Important: reset firmware's internal timer

    int pipe_to_sim[2], pipe_from_sim[2];
    pipe(pipe_to_sim); pipe(pipe_from_sim);

    pid_t pid = fork();
    if (pid == 0) {
        dup2(pipe_to_sim[0], STDIN_FILENO);
        dup2(pipe_from_sim[1], STDOUT_FILENO);
        close(pipe_to_sim[1]); close(pipe_from_sim[0]);
        // Use unbuffered python for reliability
        execlp("python3", "python3", "-u", "battery_sim.py", scenario.c_str(), (char*)NULL);
        exit(1);
    }
    close(pipe_to_sim[0]); close(pipe_from_sim[1]);

    setup();

    // Jump to 13.8V to start pulsing immediately
    ina219._v = 13.81;
    state.parasiticCurrent_mA = 50.0f;
    state.calibrated = true;
    startOutgasPulseTest();

    FILE* to_sim = fdopen(pipe_to_sim[1], "w");
    FILE* from_sim = fdopen(pipe_from_sim[0], "r");
    char line_buf[1024];

    // Accelerated simulation: dt=0.5s
    float dt = 0.5f;
    for (int i = 0; i < 5000; i++) {
        _mock_millis += (unsigned long)(dt * 1000);
        loop();

        fprintf(to_sim, "%d 0 %.3f\n", _mock_duty_ch, dt);
        fflush(to_sim);

        if (fgets(line_buf, sizeof(line_buf), from_sim)) {
            float v, i_net, v_pnl; int ntc;
            if (sscanf(line_buf, "%f %f %f %d", &v, &i_net, &v_pnl, &ntc) == 4) {
                ina219._v = v;
                ina219._i_ma = i_net;
                mock_v_pnl = v_pnl;
                _mock_ntc_counts = ntc;
            }
        }

        if (mode == MODE_CHARGE_FLOAT || mode == MODE_CHARGE_DONE) {
            printf("[%s] Characterization Complete.\n", scenario.c_str());
            break;
        }
        if (mode == MODE_FAULT) {
            printf("[%s] Fault Triggered.\n", scenario.c_str());
            break;
        }
    }

    fclose(to_sim);
    fclose(from_sim);
    kill(pid, SIGKILL);
    waitpid(pid, NULL, 0);

    return true;
}

int main(int argc, char** argv) {
    if (argc < 2) return 1;
    run_test(argv[1]);
    return 0;
}
