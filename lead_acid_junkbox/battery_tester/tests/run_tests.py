import subprocess
import time
import csv
from emulator import LeadAcidBattery

def run_simulation(duration_s=1000, dt_s=1.0, initial_soc=0.8):
    battery = LeadAcidBattery(num_cells=3, capacity_ah=2.0, initial_soc=initial_soc)

    # Start the C++ host process
    process = subprocess.Popen(
        ['./tester_host'],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        cwd='.'
    )

    results = []

    current_time_ms = 0
    measured_voltage = battery.step(0, dt_s)
    measured_current_ma = 0

    try:
        for _ in range(int(duration_s / dt_s)):
            # 1. Send measurement to controller
            input_str = f"{current_time_ms},{measured_voltage:.4f},{measured_current_ma:.2f}\n"
            process.stdin.write(input_str)
            process.stdin.flush()

            # 2. Get command from controller
            output_line = process.stdout.readline()
            if not output_line:
                break

            # Output format: time_ms,pwm,est_v,est_slope,state,target_i
            parts = output_line.strip().split(',')
            pwm = int(parts[1])
            est_v = float(parts[2])
            est_slope = float(parts[3])
            state = int(parts[4])
            target_i = float(parts[5])

            # 3. Apply PWM to physical model (simplified: PWM 255 = 1A max for this test)
            # In real system PWM might control a constant current source or just a FET.
            # Let's assume PWM 0-255 maps to 0-1000mA
            # BUT we should add some physics to the current.
            # If battery voltage is higher than charger voltage, current won't flow.
            # Let's assume charger is a 9V source.
            charger_v = 9.0
            max_i = max(0, (charger_v - measured_voltage) / 1.0) # 1 Ohm series
            actual_current_a = (pwm / 255.0) * max_i

            # 4. Step battery model
            measured_voltage = battery.step(actual_current_a, dt_s)
            # Add some noise
            import random
            measured_voltage += random.gauss(0, 0.01) # More noise
            measured_current_ma = actual_current_a * 1000.0 + random.gauss(0, 5)

            results.append({
                'time_s': current_time_ms / 1000.0,
                'voltage': measured_voltage,
                'current_ma': measured_current_ma,
                'est_v': est_v,
                'est_slope': est_slope,
                'state': state,
                'soc': battery.get_soc(),
                'pwm': pwm,
                'target_i': target_i
            })

            current_time_ms += int(dt_s * 1000)

    finally:
        process.terminate()

    return results

if __name__ == "__main__":
    print("Running simulation...")
    # Increased duration and started at higher SoC to hit gassing plateau faster
    res = run_simulation(duration_s=15000, dt_s=1.0, initial_soc=0.85)

    with open('test_results.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=res[0].keys())
        writer.writeheader()
        writer.writerows(res)
    print("Done. Results saved to test_results.csv")
