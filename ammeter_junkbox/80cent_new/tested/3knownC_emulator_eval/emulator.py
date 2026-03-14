import numpy as np

class PhysicalSystem:
    def __init__(self, voc=20.0, rint=5.0, c=0.02, rload=10.0, initial_v=None, noise_std=0.01):
        self.voc = voc
        self.rint = rint
        self.c = c
        self.rload = rload
        self.v_cap = initial_v if initial_v is not None else voc
        self.time = 0.0
        self.noise_std = noise_std

    def step(self, dt_ms, pwm_duty):
        # pwm_duty: 0.0 to 1.0
        dt = dt_ms / 1000.0

        # Analytical solution for RC circuit with two sources/resistors
        # Equivalent circuit:
        # V_eq = (Voc/Rint) / (1/Rint + pwm_duty/Rload) = Voc * Rload / (Rload + pwm_duty * Rint)
        # R_eq = 1 / (1/Rint + pwm_duty/Rload) = (Rint * Rload) / (Rload + pwm_duty * Rint)

        # Tau = R_eq * C
        # V(t) = V_eq + (V_initial - V_eq) * exp(-t/Tau)

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
