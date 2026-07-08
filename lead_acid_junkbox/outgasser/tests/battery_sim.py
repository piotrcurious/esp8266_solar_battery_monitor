import sys
import random
import math

class BatterySim:
    def __init__(self, c=100.0, r_int=0.08, outgas_v=13.9, i_parasitic=-0.050, temp=25.0):
        self.v_oc = 13.7 # Start closer to full to speed up simulation
        self.v_dl = 0.0
        self.v_gas = 0.0
        self.theta_ads = 0.0 # Surface coverage (0.0 to 1.0)

        self.c_main = c
        self.c_dl = c * 0.05

        self.r_int = r_int
        self.r_dl = r_int * 5.0

        self.base_outgas_v = outgas_v
        self.temp = temp # Celsius
        self.is_stalled = False

        # Langmuir params
        self.k_ads = 2.0
        self.k_des = 0.1
        self.v_ads_start = outgas_v - 0.20
        self.outgas_slope = 1.0
        self.i_parasitic = i_parasitic

    def get_outgas_v(self):
        return self.base_outgas_v - (self.temp - 25.0) * 0.030

    def step(self, i_applied, dt):
        # Optimized adaptive sub-stepping
        n_steps = max(1, int(dt / 0.05)) if dt > 0.05 else 1
        sub_dt = dt / n_steps

        outgas_v_now = self.get_outgas_v()
        v_ads_now = outgas_v_now - 0.20

        # Pre-cache constants for performance
        c_ads_total = self.c_main * 0.4
        dl_tc_inv = 1.0 / (self.r_dl * self.c_dl)
        oc_scale = 0.98 / self.c_main
        gas_decay = math.exp(-sub_dt / 100.0)

        for _ in range(n_steps):
            v_term_pre = self.v_oc + self.v_dl + (self.theta_ads * 0.20) + self.v_gas + i_applied * self.r_int

            # Adsorption Kinetics
            eta_ads = max(0.0, v_term_pre - v_ads_now)
            i_ads = self.k_ads * eta_ads * (1.0 - self.theta_ads) - self.k_des * self.theta_ads
            if i_ads > i_applied and i_applied > 0: i_ads = i_applied
            if i_ads < -0.2: i_ads = -0.2

            # Irreversible Gassing
            i_gas_reaction = 0
            if v_term_pre > outgas_v_now:
                i_gas_reaction = (v_term_pre - outgas_v_now)**1.5 * self.outgas_slope * (0.1 + 0.9 * self.theta_ads)
                if i_gas_reaction > (i_applied - i_ads) and i_applied > 0:
                    i_gas_reaction = max(0.0, i_applied - i_ads)

            i_charging = i_applied - i_ads - i_gas_reaction

            # State Updates
            self.v_dl += (i_applied * self.r_int - self.v_dl) * sub_dt * dl_tc_inv
            self.theta_ads = max(0.0, min(1.1, self.theta_ads + (i_ads * sub_dt) / c_ads_total))

            if i_gas_reaction > 0:
                self.v_gas += (i_gas_reaction * 0.1) * sub_dt
            else:
                self.v_gas *= gas_decay

            if not self.is_stalled:
                self.v_oc += i_charging * sub_dt * oc_scale

        v_final = self.v_oc + self.v_dl + (self.theta_ads * 0.20) + self.v_gas + i_applied * self.r_int
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
            self.cloud_factor = 0.5 + 0.5 * math.sin(self.time * 2 * math.pi / 60.0)
        elif scenario == "stormy":
            self.cloud_factor = 0.3 + 0.7 * math.sin(self.time * 2 * math.pi / 20.0)
            if self.cloud_factor < 0: self.cloud_factor = 0
        elif scenario == "dropout":
            if (int(self.time) % 10) < 5: self.cloud_factor = 0.1
            else: self.cloud_factor = 1.0
        else:
            self.cloud_factor = 1.0

    def get_i_bat(self, duty, v_bat, r_int):
        if duty < 0.01: return 0
        isc = self.isc_max * self.cloud_factor
        r_pnl = self.voc / max(0.01, isc)
        i_bat = (duty * self.voc - v_bat) / (r_int + (duty**2) * r_pnl)
        return max(0, i_bat)

def main():
    c = 10.0 # Reduced default capacity for faster simulation
    r = 0.08
    v_outgas = 13.9
    temp = 25.0
    scenario = "healthy"

    if len(sys.argv) > 1:
        scenario = sys.argv[1]
        if scenario == "high_r": r = 0.4
        elif scenario == "aged": c = 30.0; v_outgas = 13.7
        elif scenario == "sulfated": r = 0.6; c = 15.0; v_outgas = 14.5
        elif scenario == "hot": temp = 45.0
        elif scenario == "cold": temp = 0.0
        elif scenario == "stalled": v_outgas = 13.5

    bat = BatterySim(c=c, r_int=r, outgas_v=v_outgas, temp=temp)
    if scenario == "stalled": bat.is_stalled = True
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

            v_now = bat.v_oc + bat.v_dl + (bat.theta_ads * 0.20) + bat.v_gas + (-bat.i_parasitic) * bat.r_int
            i_ch = solar.get_i_bat(d_ch, v_now, bat.r_int)
            i_dis = -bat.i_parasitic
            i_net = i_ch - i_dis

            v_bat, i_actual, i_gas = bat.step(i_net, dt)

            i_pnl = d_ch * i_ch
            isc = solar.isc_max * solar.cloud_factor
            r_pnl = solar.voc / max(0.01, isc)
            v_pnl = solar.voc - i_pnl * r_pnl + (random.random() - 0.5) * 0.01
            v_pnl = max(0, v_pnl)

            r_ntc = 10000.0 * math.exp(3950.0 * (1.0/(temp+273.15) - 1.0/(25+273.15)))
            v_ntc = 3.3 * (r_ntc / (r_ntc + 10000.0))
            ntc_counts = int((v_ntc / 3.3) * 4095)

            i_net_noisy = i_net * 1000.0 + (random.random() - 0.5) * 2.0
            print(f"{v_bat:.6f} {i_net_noisy:.6f} {v_pnl:.6f} {ntc_counts}")
            sys.stdout.flush()
        except Exception:
            pass

if __name__ == "__main__":
    main()
