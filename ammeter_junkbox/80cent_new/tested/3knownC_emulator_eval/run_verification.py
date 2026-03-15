import analyzer
import os
import matplotlib.pyplot as plt

CONTROLLER = './controller'
OUT_DIR = '.'

def run_scenario(name, voc=20, rint=5, c=0.02, rload=10, duration_ms=60000, on_tick=None):
    print(f"Running scenario: {name}")
    ana = analyzer.Analyzer(CONTROLLER, voc=voc, rint=rint, c=c, rload=rload, solar_mode=True)
    ana.run(duration_ms=duration_ms, on_tick=on_tick)

    # Custom plotting for the hybrid logic
    history = ana.history
    plt.figure(figsize=(12, 10))

    plt.subplot(3, 1, 1)
    plt.plot(history['time'], history['v_cap'], label='V_cap (Physical)')
    plt.plot(history['time'], history['true_voc'], ':', alpha=0.5, label='True Voc')
    plt.plot(history['fitted_time'], history['voc_est'], '--', label='Fitted Voc (Controller)')
    plt.title(f'Unified v4.4 Hybrid Logic: {name}')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(history['time'], history['pwm_duty'], label='PWM Duty')
    plt.ylabel('Duty Cycle')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(history['time'], history['true_rint'], 'r--', alpha=0.5, label='True Rint')
    plt.plot(history['fitted_time'], history['rint_est'], label='R_src Est (Hybrid)')
    plt.xlabel('Time (s)')
    plt.ylabel('Resistance (Ohm)')
    plt.ylim(0, 20)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    filename = os.path.join(OUT_DIR, f'test_v4_4_{name.lower().replace(" ", "_")}.png')
    plt.savefig(filename)
    print(f"Saved to {filename}")

def test_cloud_transient():
    def on_tick(t_ms, emu):
        # Cloud covers at 15s (Voc drop, Rint increase)
        if 15000 < t_ms < 45000:
            emu.voc = 16.0
            emu.rint = 12.0
        # Cloud passes at 45s
        elif t_ms >= 45000:
            emu.voc = 21.0
            emu.rint = 4.0

    run_scenario("Cloud Transient", duration_ms=80000, on_tick=on_tick)

def test_static():
    run_scenario("Static Baseline", duration_ms=40000)

if __name__ == "__main__":
    test_static()
    test_cloud_transient()
