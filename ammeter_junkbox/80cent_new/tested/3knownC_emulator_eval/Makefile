# Makefile for 3knownC v4.4 Hybrid Emulator environment

CXX = g++
CXXFLAGS = -O3 -Wall -std=c++11

# Files
TARGET = controller
SRCS = controller_host.cpp 3knownC_v4_hybrid.ino mock_arduino.cpp
HEADERS = controller_logic.hpp mock_arduino.hpp

all: $(TARGET)

$(TARGET): $(SRCS) $(HEADERS)
	$(CXX) $(CXXFLAGS) -x c++ -o $(TARGET) $(SRCS)

clean:
	rm -f $(TARGET)
	rm -rf __pycache__
	rm -f *.png

run: $(TARGET)
	python3 run_verification.py

.PHONY: all clean run
