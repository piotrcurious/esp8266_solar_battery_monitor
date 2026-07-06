#include "arduino_mock.h"
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstring>
#include <signal.h>
#include <vector>
#include <string>

// Forward declaration of the mocked function
float readVinPanel();

#include "esp32_solar_battery_controller-1.cpp"

extern int _mock_duty_ch;
float mock_v_pnl = 18.0;

float readVinPanel() {
    return mock_v_pnl;
}

bool run_test(const std::string& scenario) {
    printf("\n--- STARTING TEST SCENARIO: %s ---\n", scenario.c_str());

    // Reset state
    _mock_millis = 0;
    _mock_duty_ch = 0;
    mode = MODE_BOOT;
    state = PersistedState{};

    int pipe_to_sim[2];
    int pipe_from_sim[2];
    if (pipe(pipe_to_sim) == -1 || pipe(pipe_from_sim) == -1) {
        perror("pipe");
        return false;
    }

    pid_t pid = fork();
    if (pid == 0) {
        dup2(pipe_to_sim[0], STDIN_FILENO);
        dup2(pipe_from_sim[1], STDOUT_FILENO);
        close(pipe_to_sim[1]);
        close(pipe_from_sim[0]);
        execlp("python3", "python3", "battery_sim.py", scenario.c_str(), (char*)NULL);
        perror("execlp");
        exit(1);
    }

    close(pipe_to_sim[0]);
    close(pipe_from_sim[1]);

    setup();
    startCharge();

    bool success = false;
    for (int i = 0; i < 600000; i++) {
        loop();
        _mock_millis += 100;

        char buf[256];
        sprintf(buf, "%d 0 0.1\n", _mock_duty_ch);
        write(pipe_to_sim[1], buf, strlen(buf));

        char resp[256];
        int n = read(pipe_from_sim[0], resp, sizeof(resp)-1);
        if (n > 0) {
            resp[n] = 0;
            float v, i_net, v_pnl;
            int ntc;
            if (sscanf(resp, "%f %f %f %d", &v, &i_net, &v_pnl, &ntc) == 4) {
                ina219._v = v;
                ina219._i_ma = i_net;
                mock_v_pnl = v_pnl;
                _mock_ntc_counts = ntc;
            }
        }

        if (i % 5000 == 0) {
             printf("[%s] Step %d: Mode=%d Vbatt=%.3f Ibatt=%.1f\n", scenario.c_str(), i, (int)mode, ina219._v, ina219._i_ma);
        }

        if (mode == MODE_CHARGE_DONE) {
            printf("[%s] TEST PASSED: MODE_CHARGE_DONE reached.\n", scenario.c_str());
            success = true;
            break;
        }
        if (mode == MODE_FAULT) {
            printf("[%s] TEST FAILED: MODE_FAULT reached.\n", scenario.c_str());
            break;
        }
    }

    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);
    close(pipe_to_sim[1]);
    close(pipe_from_sim[0]);

    return success;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::vector<std::string> scenarios = {"healthy", "high_r", "aged", "hot", "cold", "cloudy", "stormy", "stalled", "dropout"};
        int passed = 0;
        for (const auto& s : scenarios) {
            if (run_test(s)) passed++;
        }
        printf("\n--- SUMMARY ---\n");
        printf("Passed %d/%zu scenarios.\n", passed, scenarios.size());
        return (passed == scenarios.size()) ? 0 : 1;
    } else {
        std::string scenario = argv[1];
        return run_test(scenario) ? 0 : 1;
    }
}
