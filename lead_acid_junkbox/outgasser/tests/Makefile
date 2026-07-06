all: test_runner

esp32_solar_battery_controller-1.cpp: ../esp32_solar_battery_controller-1.ino
	cp ../esp32_solar_battery_controller-1.ino esp32_solar_battery_controller-1.cpp
	# Comment out the original readVinPanel to allow the mock to provide it
	# Use a more robust sed pattern
	sed -i '/float readVinPanel() {/,/}/ s/^/\/\//' esp32_solar_battery_controller-1.cpp

test_runner: test_runner.cpp arduino_mock.cpp esp32_solar_battery_controller-1.cpp
	g++ -I. test_runner.cpp arduino_mock.cpp -o test_runner

clean:
	rm -f test_runner esp32_solar_battery_controller-1.cpp
