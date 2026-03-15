import subprocess
import emulator
import matplotlib.pyplot as plt
import numpy as np

class Analyzer:
    def __init__(self, controller_bin, voc=20, rint=5, c=0.02, rload=10, noise_std=0.01, solar_mode=False):
        self.emulator = emulator.PhysicalSystem(voc=voc, rint=rint, c=c, rload=rload, noise_std=noise_std, solar_mode=solar_mode)
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
            'fitted_time': [],
            'true_voc': [],
            'true_rint': [],
            # Standardized names for the test scripts
            'voc_est': [],
            'rint_est': []
        }
        self.current_pwm_duty = 0.0

    def run(self, duration_ms=60000, step_ms=10, on_tick=None):
        self.process = subprocess.Popen(
            [self.controller_bin],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        current_time_ms = 0

        while current_time_ms < duration_ms:
            if on_tick:
                on_tick(current_time_ms, self.emulator)

            line = self.process.stdout.readline()
            if not line:
                print(f"Controller process exited.")
                stderr = self.process.stderr.read() if self.process.stderr else "No stderr"
                print(f"Stderr capture: {stderr}")
                break

            line = line.strip()
            if not line: continue

            if line.startswith("READ"):
                pin = int(line.split()[1])
                if pin == 0: # SOURCE_PIN (V_cap)
                    v = self.emulator.get_source_voltage()
                    raw = int(v / 25.0 * 1023)
                elif pin == 1: # CURRENT_PIN (I_src)
                    i = (self.emulator.voc - self.emulator.v_cap) / self.emulator.rint
                    i += np.random.normal(0, self.emulator.noise_std * 2) # More noise on current
                    raw = int(i / 10.0 * 1023)
                else: # LOAD_PIN
                    v = self.emulator.get_load_voltage(self.current_pwm_duty)
                    raw = int(v / 25.0 * 1023)

                self.process.stdin.write(f"{raw}\n")
                self.process.stdin.flush()

            elif line.startswith("PWM"):
                self.current_pwm_duty = float(line.split()[1])

            elif line.startswith("DATA:"):
                # Format: DATA:time_ms:v:i_src:pwm_duty:voc_est:rint_est:load_est
                parts = line.split(':')
                if len(parts) >= 8:
                    try:
                        # parts[0] is "DATA", so indices are offset by 1
                        v_est = float(parts[5])
                        r_est = float(parts[6])
                        l_est = float(parts[7])
                        self.history['fitted_voc'].append(v_est)
                        self.history['r_src_est'].append(r_est)
                        self.history['load_est'].append(l_est)
                        self.history['voc_est'].append(v_est)
                        self.history['rint_est'].append(r_est)
                        self.history['fitted_time'].append(current_time_ms / 1000.0)
                    except ValueError:
                        pass

            elif line.startswith("DELAY"):
                delay_ms = int(line.split()[1])
                n_inner_steps = max(1, delay_ms // step_ms)
                inner_dt = delay_ms / n_inner_steps
                for _ in range(n_inner_steps):
                    self.emulator.step(inner_dt, self.current_pwm_duty)
                current_time_ms += delay_ms

            elif line.startswith("SYNC"):
                # Signal the controller that it can proceed to the next iteration
                self.process.stdin.write("TICK\n")
                self.process.stdin.flush()

                parts = line.split()
                if len(parts) > 1:
                    controller_millis = int(parts[1])
                    dt = controller_millis - current_time_ms
                    if dt > 0:
                        self.emulator.step(dt, self.current_pwm_duty)
                        current_time_ms = controller_millis

                    self.history['time'].append(current_time_ms / 1000.0)
                    self.history['v_cap'].append(self.emulator.v_cap)
                    self.history['pwm_duty'].append(self.current_pwm_duty)
                    self.history['true_voc'].append(self.emulator.voc)
                    self.history['true_rint'].append(self.emulator.rint)


            elif ":" in line and "," in line:
                parts = line.split(',')
                metrics_found = False
                for part in parts:
                    if ':' in part:
                        try:
                            k, v = part.split(':', 1)
                            if k == 'V_oc':
                                self.history['fitted_voc'].append(float(v))
                                metrics_found = True
                            if k == 'Tau': self.history['fitted_tau'].append(float(v))
                            if k == 'R_src': self.history['r_src_est'].append(float(v))
                            if k == 'R_tau': self.history['r_tau_est'].append(float(v))
                            if k == 'load': self.history['load_est'].append(float(v))
                        except ValueError:
                            pass
                if metrics_found:
                    self.history['fitted_time'].append(current_time_ms / 1000.0)

        self.process.stdin.write("EXIT\n")
        self.process.terminate()

    def plot(self, filename='test_result.png', title='Simulation Results'):
        plt.figure(figsize=(12, 10))

        plt.subplot(3, 1, 1)
        plt.plot(self.history['time'], self.history['v_cap'], label='V_cap (Physical)')
        plt.plot(self.history['time'], self.history['true_voc'], label='True Voc', linestyle=':', alpha=0.5)
        if self.history['fitted_voc']:
            plt.plot(self.history['fitted_time'], self.history['fitted_voc'], label='Fitted Voc (Controller)', linestyle='--')
        plt.ylabel('Voltage (V)')
        plt.title(title)
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 2)
        plt.plot(self.history['time'], self.history['pwm_duty'], label='PWM Duty')
        if self.history['load_est']:
             plt.plot(self.history['fitted_time'], self.history['load_est'], label='Load Est', linestyle=':')
        plt.ylabel('Duty Cycle / Load')
        plt.legend()
        plt.grid(True)

        plt.subplot(3, 1, 3)
        plt.plot(self.history['time'], self.history['true_rint'], label='True Rint', color='r', linestyle='--', alpha=0.5)
        if self.history['r_src_est']:
            plt.plot(self.history['fitted_time'], self.history['r_src_est'], label='R_src Est (Iterative)')
        if self.history['r_tau_est']:
            plt.plot(self.history['fitted_time'], self.history['r_tau_est'], label='R_tau Est (From Tau)', linestyle='-.')
        plt.ylabel('Resistance (Ohm)')
        plt.xlabel('Time (s)')
        plt.legend()
        plt.grid(True)

        plt.tight_layout()
        plt.savefig(filename)
        print(f"Saved plot to {filename}")
