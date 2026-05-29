CXX = g++
CXXFLAGS = -std=c++17 -I./mock_arduino -I.. -DARDUINO=100 -g

SRCS = host_main.cpp mock_arduino/Arduino.cpp
OBJS = $(SRCS:.cpp=.o)
TARGET = sim_test

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(OBJS) $(TARGET)
