import analyzer
import os
import subprocess

CONTROLLER_SRC = 'ammeter_junkbox/80cent_new/tested/3knownC_emulator_eval/'
OUT_DIR = 'ammeter_junkbox/80cent_new/tested/3knownC_emulator_eval/'

def build_controller(logic_file):
    bin_path = os.path.join(CONTROLLER_SRC, 'controller_' + os.path.basename(logic_file).split('.')[0])
    cmd = [
        'g++', '-o', bin_path,
        os.path.join(CONTROLLER_SRC, 'controller_host.cpp'),
        os.path.join(CONTROLLER_SRC, 'mock_arduino.cpp'),
        logic_file,
        '-I', CONTROLLER_SRC
    ]
    subprocess.run(cmd, check=True)
    return bin_path

def run_suite(bin_path, name_prefix):
    # Static
    ana = analyzer.Analyzer(bin_path, voc=20, rint=5, c=0.02, rload=10)
    ana.run(duration_ms=60000)
    ana.plot(os.path.join(OUT_DIR, f'{name_prefix}_static.png'), f'{name_prefix} - Static')

    # Dynamic Rint
    ana = analyzer.Analyzer(bin_path, voc=20, rint=5, c=0.02, rload=10)
    def on_tick(t_ms, emu):
        if t_ms > 40000: emu.rint = 15.0
    ana.run(duration_ms=80000, on_tick=on_tick)
    ana.plot(os.path.join(OUT_DIR, f'{name_prefix}_dyn_rint.png'), f'{name_prefix} - Dynamic Rint')

if __name__ == "__main__":
    # Test the original "improved" version
    bin_p = build_controller(os.path.join(CONTROLLER_SRC, 'controller_logic.cpp'))
    run_suite(bin_p, 'v1_blended')

    # Test the early sampling version
    bin_p2 = build_controller(os.path.join(CONTROLLER_SRC, 'logic_early_sampling.cpp'))
    run_suite(bin_p2, 'v2_sampling')

    # Test the oscillating version
    bin_p3 = build_controller(os.path.join(CONTROLLER_SRC, 'logic_oscillating.cpp'))
    run_suite(bin_p3, 'v3_osc')
