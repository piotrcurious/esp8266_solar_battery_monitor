import analyzer

def test_oscillating_solar():
    print("Testing Buggy Oscillating Logic with Solar Mode (Non-Linear)...")
    ana = analyzer.Analyzer('./controller_oscillating', voc=20, rint=5, c=0.02, rload=10, solar_mode=True)
    ana.run(duration_ms=60000)
    ana.plot(filename='test_oscillating_solar.png', title='Buggy Oscillating Logic Solar Mode Analysis')

if __name__ == "__main__":
    test_oscillating_solar()
