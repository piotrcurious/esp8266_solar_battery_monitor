import analyzer

def test_unified_solar():
    print("Testing Unified Logic with Solar Mode (Non-Linear) + Transients...")
    ana = analyzer.Analyzer('./controller', voc=20, rint=5, c=0.02, rload=10, solar_mode=True)

    def inject_transient(t_ms, emu):
        if t_ms > 30000 and t_ms < 31000:
            emu.voc = 15.0 # Sudden cloud
        if t_ms >= 31000:
            emu.voc = 15.0

    ana.run(duration_ms=60000, on_tick=inject_transient)
    ana.plot(filename='test_unified_solar_transient.png', title='Unified Logic (v3.1) Solar Transient Analysis')

if __name__ == "__main__":
    test_unified_solar()
