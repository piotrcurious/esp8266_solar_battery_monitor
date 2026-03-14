import analyzer
import os

CONTROLLER = './ammeter_junkbox/80cent_new/tested/3knownC_emulator_eval/controller'
OUT_DIR = 'ammeter_junkbox/80cent_new/tested/3knownC_emulator_eval/'

def test_static():
    ana = analyzer.Analyzer(CONTROLLER, voc=20, rint=5, c=0.02, rload=10)
    ana.run(duration_ms=60000)
    ana.plot(os.path.join(OUT_DIR, 'test_static.png'), 'Static Conditions (Voc=20, Rint=5)')

def test_dynamic_voc():
    ana = analyzer.Analyzer(CONTROLLER, voc=20, rint=5, c=0.02, rload=10)
    def on_tick(t_ms, emu):
        # Drop Voc from 20 to 15 at 30s
        if t_ms > 30000:
            emu.voc = 15.0
    ana.run(duration_ms=60000, on_tick=on_tick)
    ana.plot(os.path.join(OUT_DIR, 'test_dynamic_voc.png'), 'Dynamic Voc (20V -> 15V at 30s)')

def test_dynamic_rint():
    ana = analyzer.Analyzer(CONTROLLER, voc=20, rint=5, c=0.02, rload=10)
    def on_tick(t_ms, emu):
        # Increase Rint from 5 to 15 at 40s
        if t_ms > 40000:
            emu.rint = 15.0
    ana.run(duration_ms=80000, on_tick=on_tick)
    ana.plot(os.path.join(OUT_DIR, 'test_dynamic_rint.png'), 'Dynamic Rint (5 Ohm -> 15 Ohm at 40s)')

def test_high_noise():
    ana = analyzer.Analyzer(CONTROLLER, voc=20, rint=5, c=0.02, rload=10, noise_std=0.1)
    ana.run(duration_ms=60000)
    ana.plot(os.path.join(OUT_DIR, 'test_high_noise.png'), 'High Noise (std=0.1)')

if __name__ == "__main__":
    test_static()
    test_dynamic_voc()
    test_dynamic_rint()
    test_high_noise()
