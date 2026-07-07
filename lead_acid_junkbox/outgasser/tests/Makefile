SCENARIOS = healthy high_r aged cloudy hot cold stalled stormy dropout sulfated
LOGS = $(addsuffix .log, $(SCENARIOS))
SVGS_FULL = $(addsuffix _full.svg, $(SCENARIOS))
SVGS_PULSES = $(addsuffix _pulses.svg, $(SCENARIOS))

all: math_unit_test $(SVGS_FULL) $(SVGS_PULSES)

math_unit_test: math_unit_test.cpp
	g++ math_unit_test.cpp -o math_unit_test
	./math_unit_test

%.log: test_runner
	./test_runner $* > $@ 2>&1 || true

%_full.svg %_pulses.svg: %.log plot_results.py
	python3 plot_results.py $< $*

esp32_solar_battery_controller-1.cpp: ../esp32_solar_battery_controller-1.ino
	cp ../esp32_solar_battery_controller-1.ino esp32_solar_battery_controller-1.cpp
	# Comment out the original readVinPanel to allow the mock to provide it
	# Use a more robust sed pattern
	sed -i '/float readVinPanel() {/,/}/ s/^/\/\//' esp32_solar_battery_controller-1.cpp

test_runner: test_runner.cpp arduino_mock.cpp esp32_solar_battery_controller-1.cpp
	g++ -I. test_runner.cpp arduino_mock.cpp -o test_runner

clean:
	rm -f test_runner math_unit_test esp32_solar_battery_controller-1.cpp *.log *.svg

.PHONY: all clean math_unit_test
