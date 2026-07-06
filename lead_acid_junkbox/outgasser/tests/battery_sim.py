import sys
import random

# Simulation parameters
# No real-time sleep, we run as fast as possible for testing.

class BatterySim:
    def __init__(self):
        self.v_cap = 13.5 # Starting near bulk
        self.c = 100.0 # Even smaller C for faster simulation
        self.r_int = 0.08 # Ohms
        self.outgas_v = 13.9 # Lower outgas_v to reach it sooner
        self.outgas_slope = 1.0 # A/V
        self.i_parasitic = -0.050 # 50mA discharge

    def step(self, i_applied, dt):
        # i_applied is current into battery (A)
        # V_term = v_cap + i_applied * r_int
        v_term = self.v_cap + i_applied * self.r_int

        i_gas = 0
        if v_term > self.outgas_v:
            i_gas = (v_term - self.outgas_v) * self.outgas_slope
            i_gas = min(i_gas, max(0.0, i_applied))

        i_cap = i_applied - i_gas
        self.v_cap += (i_cap * dt) / self.c

        v_term = self.v_cap + i_applied * self.r_int + (random.random() - 0.5) * 0.001
        return v_term, i_applied, i_gas

class SolarSim:
    def __init__(self):
        self.voc = 20.0
        self.isc = 2.5
        self.r_pnl = self.voc / self.isc

    def get_i_bat(self, duty, v_cap, r_int):
        if duty < 0.01: return 0
        # I_bat = (D*Voc - Vcap) / (Rint + D^2 * Rpnl)
        i_bat = (duty * self.voc - v_cap) / (r_int + (duty**2) * self.r_pnl)
        return max(0, i_bat)

def main():
    bat = BatterySim()
    solar = SolarSim()

    while True:
        line = sys.stdin.readline()
        if not line: break
        try:
            parts = line.split()
            if len(parts) < 3: continue
            duty_ch = float(parts[0])
            duty_dis = float(parts[1])
            dt = float(parts[2])

            d_ch = duty_ch / 255.0
            d_dis = duty_dis / 255.0

            i_ch = solar.get_i_bat(d_ch, bat.v_cap, bat.r_int)
            # Use fixed parasitic load instead of d_dis
            i_dis = -bat.i_parasitic # +ve i_dis is discharge

            i_net = i_ch - i_dis

            v_bat, i_actual, i_gas = bat.step(i_net, dt)

            i_pnl = d_ch * i_ch
            v_pnl = solar.voc - i_pnl * solar.r_pnl + (random.random() - 0.5) * 0.01
            v_pnl = max(0, v_pnl)

            i_net_noisy = i_net * 1000.0 + (random.random() - 0.5) * 2.0

            print(f"{v_bat:.6f} {i_net_noisy:.6f} {v_pnl:.6f}")
            sys.stdout.flush()
        except Exception:
            pass

if __name__ == "__main__":
    main()
