import pandas as pd
import sys

def analyze(filename):
    df = pd.read_csv(filename)
    print(df.describe())
    print("\nLast 10 rows:")
    print(df.tail(10))

    # Check why it didn't trigger
    # Trigger condition: (voltage_est > 7.0) && (voltageSlope < 0.0002) && (voltageSlope > -0.00001)
    # after 30s
    condition = (df['est_v'] > 7.0) & (df['est_slope'] < 0.0002) & (df['est_slope'] > -0.00001)
    triggered_count = condition.sum()
    print(f"\nCondition met in {triggered_count} rows")
    if triggered_count > 0:
        print("First time condition met:")
        print(df[condition].iloc[0])

if __name__ == "__main__":
    analyze(sys.argv[1])
