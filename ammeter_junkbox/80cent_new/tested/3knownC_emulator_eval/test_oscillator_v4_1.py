import analyzer
import matplotlib.pyplot as plt

def test_oscillator_v4_1():
    print("Testing Oscillator Logic (v4.2 - Binning) with Solar Mode...")
    # The compiled binary is controller_oscillator_v4_2
    ana = analyzer.Analyzer('./controller_oscillator_v4_2', voc=20, rint=5, c=0.02, rload=10, solar_mode=True)

    def inject_transient(t_ms, emu):
        # 1. Voc Drop at 15s (Partial Shading)
        if 15000 < t_ms <= 30000:
            emu.voc = 17.0
        elif t_ms > 30000:
            emu.voc = 19.0

        # 2. Rint Increase at 45s (Cloud Edge / High Resistance)
        if t_ms > 45000:
            emu.rint = 12.0
        elif t_ms > 10000:
            emu.rint = 6.0

    ana.run(duration_ms=80000, on_tick=inject_transient)

    # Custom plotting to fix limits
    history = ana.history
    plt.figure(figsize=(12, 10))

    plt.subplot(3, 1, 1)
    plt.plot(history['time'], history['v_cap'], label='V_cap (Physical)')
    plt.plot(history['time'], history['true_voc'], ':', alpha=0.5, label='True Voc')
    plt.plot(history['fitted_time'], history['voc_est'], '--', label='Fitted Voc (Controller)')
    plt.title('Software RC Oscillator Logic (v4.2) Solar Analysis')
    plt.ylabel('Voltage (V)')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(history['time'], history['pwm_duty'], label='PWM Duty')
    plt.plot(history['fitted_time'], history['load_est'], ':', label='Load Est')
    plt.ylabel('Duty Cycle / Load')
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(history['time'], history['true_rint'], 'r--', alpha=0.5, label='True Rint')
    plt.plot(history['fitted_time'], history['rint_est'], label='R_src Est (Iterative)')
    plt.xlabel('Time (s)')
    plt.ylabel('Resistance (Ohm)')
    plt.ylim(0, 15)
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.savefig('test_oscillator_v4_2.png')
    print("Saved results to test_oscillator_v4_2.png")

if __name__ == "__main__":
    test_oscillator_v4_1()
