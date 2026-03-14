CXX = g++
CXXFLAGS = -I. -Itests -DARDUINO=100
TARGET = tests/filter_test
SRC = SensorFilter.cpp tests/test_runner.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
