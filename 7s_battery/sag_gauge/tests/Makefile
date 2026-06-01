CXX = g++
CXXFLAGS = -std=c++17 -I./mock_arduino -I.. -DARDUINO=100 -g
MOCK_OBJ = mock_arduino/Arduino.o

TESTS = math_test fuzz_test robustness_test ui_comprehensive_test ui_logic_test ui_debug_test nvs_test perf_test stress_test

all: $(MOCK_OBJ) $(TESTS)

mock_arduino/Arduino.o: mock_arduino/Arduino.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

%: %.cpp $(MOCK_OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(TESTS) *.o mock_arduino/*.o *.ppm
