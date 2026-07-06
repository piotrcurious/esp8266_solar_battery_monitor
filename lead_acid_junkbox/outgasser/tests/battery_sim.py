import sys
import random
import math

class BatterySim:
    def __init__(self, c=100.0, r_int=0.08, outgas_v=13.9, i_parasitic=-0.050, temp=25.0):
        self.v_oc = 13.5
        self.v_dl = 0.0
        self.v_gas = 0.0

        self.c_main = c
        self.c_dl = c * 0.05
        self.c_gas = c * 2.0

        self.r_int = r_int
        self.r_dl = r_int * 5.0
        self.r_gas = r_int * 100.0

        self.base_outgas_v = outgas_v
        self.outgas_slope = 1.0
        self.i_parasitic = i_parasitic
        self.temp = temp # Celsius
        self.is_stalled = False

    def get_outgas_v(self):
        # Lead acid outgassing voltage drops ~30mV per degree C above 25C
        return self.base_outgas_v - (self.temp - 25.0) * 0.030

    def step(self, i_applied, dt):
        n_steps = 10
        sub_dt = dt / n_steps

        outgas_v_now = self.get_outgas_v()

        for _ in range(n_steps):
            v_sum = self.v_oc + self.v_dl + self.v_gas
            v_term_act = v_sum + i_applied * self.r_int

            i_gas_reaction = 0
            if v_term_act > outgas_v_now:
                i_gas_reaction = (v_term_act - outgas_v_now) * self.outgas_slope
                i_gas_reaction = min(i_gas_reaction, max(0.0, i_applied))

            i_charging = i_applied - i_gas_reaction

            self.v_dl += (i_applied * self.r_int - self.v_dl) * (sub_dt / (self.r_dl * self.c_dl))

            if i_gas_reaction > 0:
                self.v_gas += (i_gas_reaction * 0.2) * (sub_dt / self.c_gas)
            else:
                self.v_gas *= math.exp(-sub_dt / 50.0)

            if not self.is_stalled:
                self.v_oc += (i_charging * 0.95 * sub_dt) / self.c_main

        v_final = self.v_oc + self.v_dl + self.v_gas + i_applied * self.r_int
        v_final += (random.random() - 0.5) * 0.001

        return v_final, i_applied, i_gas_reaction

class SolarSim:
    def __init__(self, voc=20.0, isc_max=2.5):
        self.voc = voc
        self.isc_max = isc_max
        self.time = 0.0
        self.cloud_factor = 1.0

    def update_clouds(self, dt, scenario):
        self.time += dt
        if scenario == "cloudy":
            # Cloud passing every 60s
            self.cloud_factor = 0.5 + 0.5 * math.sin(self.time * 2 * math.pi / 60.0)
        else:
            self.cloud_factor = 1.0

    def get_i_bat(self, duty, v_bat, r_int):
        if duty < 0.01: return 0
        isc = self.isc_max * self.cloud_factor
        r_pnl = self.voc / max(0.01, isc)
        i_bat = (duty * self.voc - v_bat) / (r_int + (duty**2) * r_pnl)
        return max(0, i_bat)

def main():
    c = 100.0
    r = 0.08
    v_outgas = 13.9
    temp = 25.0
    scenario = "healthy"

    if len(sys.argv) > 1:
        scenario = sys.argv[1]
        if scenario == "high_r":
            r = 0.4
        elif scenario == "aged":
            c = 30.0
            v_outgas = 13.7
        elif scenario == "hot":
            temp = 45.0
        elif scenario == "cold":
            temp = 0.0
        elif scenario == "stalled":
            v_outgas = 13.5

    bat = BatterySim(c=c, r_int=r, outgas_v=v_outgas, temp=temp)
    if scenario == "stalled":
        bat.is_stalled = True
    solar = SolarSim()

    while True:
        line = sys.stdin.readline()
        if not line: break
        try:
            parts = line.split()
            if len(parts) < 3: continue
            duty_ch = float(parts[0])
            dt = float(parts[2])

            solar.update_clouds(dt, scenario)

            d_ch = duty_ch / 255.0

            v_now = bat.v_oc + bat.v_dl + bat.v_gas + (-bat.i_parasitic) * bat.r_int
            i_ch = solar.get_i_bat(d_ch, v_now, bat.r_int)
            i_dis = -bat.i_parasitic

            i_net = i_ch - i_dis

            v_bat, i_actual, i_gas = bat.step(i_net, dt)

            i_pnl = d_ch * i_ch
            isc = solar.isc_max * solar.cloud_factor
            r_pnl = solar.voc / max(0.01, isc)
            v_pnl = solar.voc - i_pnl * r_pnl + (random.random() - 0.5) * 0.01
            v_pnl = max(0, v_pnl)

            # Simple NTC mock: 10k at 25C, beta=3950
            r_ntc = 10000.0 * math.exp(3950.0 * (1.0/(temp+273.15) - 1.0/(25+273.15)))
            # Voltage divider with 10k fixed
            v_ntc = 3.3 * (r_ntc / (r_ntc + 10000.0))
            ntc_counts = int((v_ntc / 3.3) * 4095)

            i_net_noisy = i_net * 1000.0 + (random.random() - 0.5) * 2.0

            print(f"{v_bat:.6f} {i_net_noisy:.6f} {v_pnl:.6f} {ntc_counts}")
            sys.stdout.flush()
        except Exception:
            pass

if __name__ == "__main__":
    main()
