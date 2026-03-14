import subprocess
import emulator
import matplotlib.pyplot as plt
import numpy as np

class Analyzer:
    def __init__(self, controller_bin, voc=20, rint=5, c=0.02, rload=10, noise_std=0.01):
        self.emulator = emulator.PhysicalSystem(voc=voc, rint=rint, c=c, rload=rload, noise_std=noise_std)
        self.controller_bin = controller_bin
        self.process = None
        self.history = {
            'time': [],
            'v_cap': [],
            'v_src': [],
            'v_load': [],
            'pwm_duty': [],
            'fitted_voc': [],
            'fitted_tau': [],
            'r_src_est': [],
            'r_tau_est': [],
            'load_est': [],
            'true_voc': [],
            'true_rint': []
        }
        self.current_pwm_duty = 0.0

    def run(self, duration_ms=60000, step_ms=10, on_tick=None):
        self.process = subprocess.Popen(
            [self.controller_bin],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        current_time_ms = 0

        while current_time_ms < duration_ms:
            if on_tick:
                on_tick(current_time_ms, self.emulator)

            line = self.process.stdout.readline()
            if not line:
                break

            line = line.strip()

            if line.startswith("READ"):
                pin = int(line.split()[1])
                if pin == 0: # SOURCE_PIN
                    v = self.emulator.get_source_voltage()
                    raw = int(v / 28.0 * 1023)
                else: # LOAD_PIN
                    v = self.emulator.get_load_voltage(self.current_pwm_duty)
                    raw = int(v / 30.0 * 1023)
                self.process.stdin.write(f"{raw}\n")
                self.process.stdin.flush()

            elif line.startswith("PWM"):
                self.current_pwm_duty = float(line.split()[1])

            elif line.startswith("DELAY"):
                delay_ms = int(line.split()[1])
                n_inner_steps = max(1, delay_ms // step_ms)
                inner_dt = delay_ms / n_inner_steps
                for _ in range(n_inner_steps):
                    self.emulator.step(inner_dt, self.current_pwm_duty)
                current_time_ms += delay_ms

            elif line.startswith("SYNC"):
                controller_millis = int(line.split()[1])
                dt = controller_millis - current_time_ms
                if dt > 0:
                    self.emulator.step(dt, self.current_pwm_duty)
                    current_time_ms = controller_millis

                self.history['time'].append(current_time_ms / 1000.0)
                self.history['v_cap'].append(self.emulator.v_cap)
                self.history['pwm_duty'].append(self.current_pwm_duty)
                self.history['true_voc'].append(self.emulator.voc)
                self.history['true_rint'].append(self.emulator.rint)

                self.process.stdin.write(f"TICK {current_time_ms + step_ms}\n")
                self.process.stdin.flush()

            elif ":" in line and "," in line:
                parts = line.split(',')
                for part in parts:
                    if ':' in part:
                        try:
                            k, v = part.split(':', 1)
                            if k == 'V_oc': self.history['fitted_voc'].append(float(v))
                            if k == 'Tau': self.history['fitted_tau'].append(float(v))
                            if k == 'R_src': self.history['r_src_est'].append(float(v))
                            if k == 'R_tau': self.history['r_tau_est'].append(float(v))
                            if k == 'load': self.history['load_est'].append(float(v))
                        except ValueError:
                            pass

        self.process.stdin.write("EXIT\n")
        self.process.terminate()

    def plot(self, filename='test_result.png', title='Simulation Results'):
        plt.figure(figsize=(12, 10))

        plt.subplot(3, 1, 1)
        plt.plot(self.history['time'], self.history['v_cap'], label='V_cap (Physical)')
        plt.plot(self.history['time'], self.history['true_voc'], label='True Voc', linestyle=':', alpha=0.5)
        if self.history['fitted_voc']:
            t_fitted = np.linspace(0, self.history['time'][-1], len(self.history['fitted_voc']))
            plt.plot(t_fitted, self.history['fitted_voc'], label='Fitted Voc (Controller)', linestyle='--')
        plt.ylabel('Voltage (V)')
        plt.title(title)
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(self.history['time'], self.history['pwm_duty'], label='PWM Duty')
        if self.history['load_est']:
             t_load = np.linspace(0, self.history['time'][-1], len(self.history['load_est']))
             plt.plot(t_load, self.history['load_est'], label='Load Est', linestyle=':')
        plt.ylabel('Duty Cycle / Load')
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(self.history['time'], self.history['true_rint'], label='True Rint', color='r', linestyle='--', alpha=0.5)
        if self.history['r_src_est']:
            t_r = np.linspace(0, self.history['time'][-1], len(self.history['r_src_est']))
            plt.plot(t_r, self.history['r_src_est'], label='R_src Est (Iterative)')
        if self.history['r_tau_est']:
            t_rt = np.linspace(0, self.history['time'][-1], len(self.history['r_tau_est']))
            plt.plot(t_rt, self.history['r_tau_est'], label='R_tau Est (From Tau)', linestyle='-.')
        plt.ylabel('Resistance (Ohm)')
        plt.xlabel('Time (s)')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig(filename)
        print(f"Saved plot to {filename}")
