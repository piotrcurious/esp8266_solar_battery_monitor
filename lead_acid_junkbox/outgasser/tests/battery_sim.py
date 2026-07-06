import sys
import random
import math

class BatterySim:
    def __init__(self, c=100.0, r_int=0.08, outgas_v=13.9, i_parasitic=-0.050):
        self.v_oc = 13.5
        self.v_dl = 0.0
        self.v_gas = 0.0

        self.c_main = c
        self.c_dl = c * 0.05
        self.c_gas = c * 2.0

        self.r_int = r_int
        self.r_dl = r_int * 5.0
        self.r_gas = r_int * 100.0

        self.outgas_v_thresh = outgas_v
        self.outgas_slope = 1.0
        self.i_parasitic = i_parasitic

    def step(self, i_applied, dt):
        # Sub-stepping for stability
        n_steps = 10
        sub_dt = dt / n_steps

        for _ in range(n_steps):
            # Combined voltage
            v_sum = self.v_oc + self.v_dl + self.v_gas
            v_term_act = v_sum + i_applied * self.r_int

            # Outgassing current
            i_gas_reaction = 0
            if v_term_act > self.outgas_v_thresh:
                i_gas_reaction = (v_term_act - self.outgas_v_thresh) * self.outgas_slope
                i_gas_reaction = min(i_gas_reaction, max(0.0, i_applied))

            i_charging = i_applied - i_gas_reaction

            # Update components
            # Double layer responds to i_applied (mostly)
            self.v_dl += (i_applied * self.r_int - self.v_dl) * (sub_dt / (self.r_dl * self.c_dl))

            # Gas component builds up when outgassing
            if i_gas_reaction > 0:
                self.v_gas += (i_gas_reaction * 0.2) * (sub_dt / self.c_gas)
            else:
                self.v_gas *= math.exp(-sub_dt / 50.0)

            # Bulk OC
            self.v_oc += (i_charging * 0.95 * sub_dt) / self.c_main

        v_final = self.v_oc + self.v_dl + self.v_gas + i_applied * self.r_int
        v_final += (random.random() - 0.5) * 0.001

        return v_final, i_applied, i_gas_reaction

class SolarSim:
    def __init__(self, voc=20.0, isc=2.5):
        self.voc = voc
        self.isc = isc
        self.r_pnl = self.voc / self.isc

    def get_i_bat(self, duty, v_bat, r_int):
        if duty < 0.01: return 0
        i_bat = (duty * self.voc - v_bat) / (r_int + (duty**2) * self.r_pnl)
        return max(0, i_bat)

def main():
    c = 100.0
    r = 0.08
    v_outgas = 13.9

    if len(sys.argv) > 1:
        scenario = sys.argv[1]
        if scenario == "high_r":
            r = 0.4
        elif scenario == "aged":
            c = 30.0
            v_outgas = 13.7

    bat = BatterySim(c=c, r_int=r, outgas_v=v_outgas)
    solar = SolarSim()

    while True:
        line = sys.stdin.readline()
        if not line: break
        try:
            parts = line.split()
            if len(parts) < 3: continue
            duty_ch = float(parts[0])
            dt = float(parts[2])

            d_ch = duty_ch / 255.0

            # Current battery voltage for solar sim
            v_now = bat.v_oc + bat.v_dl + bat.v_gas + (-bat.i_parasitic) * bat.r_int
            i_ch = solar.get_i_bat(d_ch, v_now, bat.r_int)
            i_dis = -bat.i_parasitic

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
