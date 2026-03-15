import csv
import matplotlib.pyplot as plt

def plot_results(csv_file):
    times = []
    voltages = []
    currents = []
    est_vs = []
    est_slopes = []
    states = []
    socs = []
    target_is = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            times.append(float(row['time_s']))
            voltages.append(float(row['voltage']))
            currents.append(float(row['current_ma']))
            est_vs.append(float(row['est_v']))
            est_slopes.append(float(row['est_slope']))
            states.append(int(row['state']))
            socs.append(float(row['soc']))
            target_is.append(float(row['target_i']))

    fig, axs = plt.subplots(4, 1, figsize=(12, 16), sharex=True)

    # Voltage
    axs[0].plot(times, voltages, label='Measured Voltage', alpha=0.5)
    axs[0].plot(times, est_vs, label='Estimated Voltage (KF)', color='red')
    axs[0].set_ylabel('Voltage (V)')
    axs[0].legend()
    axs[0].grid(True)
    axs[0].set_title('Battery Tester Performance')

    # Current
    axs[1].plot(times, currents, label='Measured Current', alpha=0.5)
    axs[1].plot(times, target_is, label='Target Current', linestyle='--')
    axs[1].set_ylabel('Current (mA)')
    axs[1].legend()
    axs[1].grid(True)

    # Slope
    axs[2].plot(times, est_slopes, label='Voltage Slope (V/s)', color='green')
    axs[2].axhline(y=0, color='black', linestyle='-', alpha=0.3)
    axs[2].set_ylabel('Slope (V/s)')
    axs[2].legend()
    axs[2].grid(True)

    # State and SoC
    ax4_2 = axs[3].twinx()
    axs[3].plot(times, states, label='State', color='purple')
    ax4_2.plot(times, socs, label='SoC', color='orange', linestyle=':')
    axs[3].set_ylabel('State')
    ax4_2.set_ylabel('SoC')
    axs[3].set_xlabel('Time (s)')
    axs[3].legend(loc='upper left')
    ax4_2.legend(loc='upper right')
    axs[3].grid(True)

    plt.tight_layout()
    plt.savefig('test_results.png')
    print("Graph saved to test_results.png")

if __name__ == "__main__":
    plot_results('test_results.csv')
