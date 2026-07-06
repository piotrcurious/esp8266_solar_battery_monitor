#include "arduino_mock.h"
#include <unistd.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <cstring>
#include <signal.h>

// Forward declaration of the mocked function
float readVinPanel();

#include "esp32_solar_battery_controller-1.cpp"

extern int _mock_duty_ch;
extern int _mock_duty_dis;

float mock_v_pnl = 18.0;

float readVinPanel() {
    return mock_v_pnl;
}

int main() {
    // Start the Python simulator
    int pipe_to_sim[2];
    int pipe_from_sim[2];
    if (pipe(pipe_to_sim) == -1 || pipe(pipe_from_sim) == -1) {
        perror("pipe");
        return 1;
    }

    pid_t pid = fork();
    if (pid == 0) {
        // Child: exec python sim
        dup2(pipe_to_sim[0], STDIN_FILENO);
        dup2(pipe_from_sim[1], STDOUT_FILENO);
        close(pipe_to_sim[1]);
        close(pipe_from_sim[0]);
        execlp("python3", "python3", "battery_sim.py", (char*)NULL);
        perror("execlp");
        exit(1);
    }

    close(pipe_to_sim[0]);
    close(pipe_from_sim[1]);

    setup();

    // Start charging
    startCharge();

    for (int i = 0; i < 500000; i++) { // Run for more ticks
        loop();
        _mock_millis += 100; // Increment mock time (CONTROL_INTERVAL_MS)

        // Sync with simulator every tick
        char buf[256];
        sprintf(buf, "%d %d 0.1\n", _mock_duty_ch, _mock_duty_dis);
        write(pipe_to_sim[1], buf, strlen(buf));

        // Read back from simulator
        char resp[256];
        int n = read(pipe_from_sim[0], resp, sizeof(resp)-1);
        if (n > 0) {
            resp[n] = 0;
            float v, i_net, v_pnl;
            if (sscanf(resp, "%f %f %f", &v, &i_net, &v_pnl) == 3) {
                ina219._v = v;
                ina219._i_ma = i_net;
                mock_v_pnl = v_pnl;
            }
        }

        if (i % 1000 == 0) {
             printf("Step %d: Mode=%d Vbatt=%.3f Ibatt=%.1f duty=%d\n", i, (int)mode, ina219._v, ina219._i_ma, _mock_duty_ch);
        }

        if (mode == MODE_CHARGE_DONE) {
            printf("\nTEST PASSED: MODE_CHARGE_DONE reached.\n");
            break;
        }
    }

    kill(pid, SIGTERM);
    waitpid(pid, NULL, 0);

    return 0;
}
