SCENARIOS = healthy high_r aged cloudy hot cold
LOGS = $(addsuffix .log, $(SCENARIOS))
SVGS = $(addsuffix _full.svg, $(SCENARIOS)) $(addsuffix _pulses.svg, $(SCENARIOS))

all: $(SVGS)

%.log: test_runner
	./test_runner $* > $@ 2>&1 || true

%_full.svg %_pulses.svg: %.log plot_results.py
	python3 plot_results.py $< $*

esp32_solar_battery_controller-1.cpp: ../esp32_solar_battery_controller-1.ino
	cp ../esp32_solar_battery_controller-1.ino esp32_solar_battery_controller-1.cpp
	sed -i '/float readVinPanel() {/,/}/ s/^/\/\//' esp32_solar_battery_controller-1.cpp

test_runner: test_runner.cpp arduino_mock.cpp esp32_solar_battery_controller-1.cpp
	g++ -I. test_runner.cpp arduino_mock.cpp -o test_runner

clean:
	rm -f test_runner esp32_solar_battery_controller-1.cpp *.log *.svg

.PHONY: all clean
