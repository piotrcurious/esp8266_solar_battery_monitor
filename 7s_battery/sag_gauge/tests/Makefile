CXX = g++
CXXFLAGS = -std=c++17 -I./mock_arduino -I.. -DARDUINO=100 -g

SRCS = host_main.cpp mock_arduino/Arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = sim_test
STRESS = stress_test

all: $(TARGET) $(STRESS)

$(TARGET): host_main.o mock_arduino/Arduino.o
	$(CXX) $(CXXFLAGS) -o $@ $^

$(STRESS): stress_test.o mock_arduino/Arduino.o
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(OBJS) $(TARGET)
