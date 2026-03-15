import numpy as np

class PhysicalSystem:
    def __init__(self, voc=20.0, rint=5.0, c=0.02, rload=10.0, initial_v=None, noise_std=0.01, solar_mode=False):
        self.voc = voc
        self.rint_base = rint
        self.rint = rint
        self.c = c
        self.rload = rload
        self.v_cap = initial_v if initial_v is not None else voc
        self.time = 0.0
        self.noise_std = noise_std
        self.solar_mode = solar_mode

    def step(self, dt_ms, pwm_duty):
        # pwm_duty: 0.0 to 1.0
        dt = dt_ms / 1000.0

        # In solar mode, Rint increases as voltage drops (simulating the knee of the IV curve)
        if self.solar_mode:
            voltage_ratio = self.v_cap / self.voc
            # Exponentially increase resistance as we pull more current (voltage drops)
            self.rint = self.rint_base * (1.0 + 10.0 * np.exp(-10.0 * voltage_ratio))
        else:
            self.rint = self.rint_base

        # Analytical solution for RC circuit with two sources/resistors
        denom = self.rload + pwm_duty * self.rint
        if denom == 0: # Should not happen if Rload > 0
            v_eq = self.voc
            r_eq = self.rint
        else:
            v_eq = (self.voc * self.rload) / denom
            r_eq = (self.rint * self.rload) / denom

        tau = r_eq * self.c
        if tau > 0:
            self.v_cap = v_eq + (self.v_cap - v_eq) * np.exp(-dt / tau)
        else:
            self.v_cap = v_eq

        self.time += dt
        return self.v_cap

    def get_source_voltage(self):
        return self.v_cap + np.random.normal(0, self.noise_std)

    def get_load_voltage(self, pwm_duty):
        # Assuming LOAD_PIN measures average voltage after the PWM switch
        return (self.v_cap * pwm_duty) + np.random.normal(0, self.noise_std)

if __name__ == "__main__":
    # Test charging
    sys = PhysicalSystem(voc=20, rint=10, c=0.01, rload=10)
    sys.v_cap = 0 # Start empty
    print("Charging...")
    for i in range(10):
        v = sys.step(100, 0.0) # 100ms steps, 0 duty
        print(f"Time: {sys.time:.2f}s, V: {v:.4f}V")

    # Test discharging/loading
    print("\nLoading...")
    for i in range(10):
        v = sys.step(100, 0.5) # 100ms steps, 0.5 duty
        print(f"Time: {sys.time:.2f}s, V: {v:.4f}V")
