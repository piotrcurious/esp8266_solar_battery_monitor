import analyzer

def test_oscillating():
    print("Testing Oscillating Logic...")
    ana = analyzer.Analyzer('./controller_osc', voc=20, rint=5, c=0.02, rload=10, solar_mode=True)
    ana.run(duration_ms=60000)
    ana.plot(filename='test_oscillating_bug.png', title='Oscillating Logic Bug Analysis')

if __name__ == "__main__":
    test_oscillating()
