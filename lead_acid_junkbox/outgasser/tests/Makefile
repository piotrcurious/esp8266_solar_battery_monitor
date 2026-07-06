all: healthy_results.svg high_r_results.svg aged_results.svg cloudy_results.svg

# Scenario specific logs
healthy.log: test_runner
	./test_runner healthy > healthy.log 2>&1 || true

high_r.log: test_runner
	./test_runner high_r > high_r.log 2>&1 || true

aged.log: test_runner
	./test_runner aged > aged.log 2>&1 || true

cloudy.log: test_runner
	./test_runner cloudy > cloudy.log 2>&1 || true

# SVG generation
healthy_results.svg: healthy.log plot_results.py
	python3 plot_results.py healthy.log healthy_results.svg "Healthy Battery Charge Cycle"

high_r_results.svg: high_r.log plot_results.py
	python3 plot_results.py high_r.log high_r_results.svg "High Resistance Battery"

aged_results.svg: aged.log plot_results.py
	python3 plot_results.py aged.log aged_results.svg "Aged (Low Capacity) Battery"

cloudy_results.svg: cloudy.log plot_results.py
	python3 plot_results.py cloudy.log cloudy_results.svg "Variable Solar (Cloudy) Conditions"

# Original build rules
esp32_solar_battery_controller-1.cpp: ../esp32_solar_battery_controller-1.ino
	cp ../esp32_solar_battery_controller-1.ino esp32_solar_battery_controller-1.cpp
	# Comment out the original readVinPanel to allow the mock to provide it
	# Use a more robust sed pattern
	sed -i '/float readVinPanel() {/,/}/ s/^/\/\//' esp32_solar_battery_controller-1.cpp

test_runner: test_runner.cpp arduino_mock.cpp esp32_solar_battery_controller-1.cpp
	g++ -I. test_runner.cpp arduino_mock.cpp -o test_runner

clean:
	rm -f test_runner esp32_solar_battery_controller-1.cpp *.log *.svg
