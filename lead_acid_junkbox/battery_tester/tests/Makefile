CC=g++
CFLAGS=-I../src -Wall
TARGET=tester_host

all: $(TARGET)

$(TARGET): host_main.cpp ../src/BatteryTester.cpp
	$(CC) $(CFLAGS) -o $(TARGET) host_main.cpp ../src/BatteryTester.cpp

clean:
	rm -f $(TARGET)
