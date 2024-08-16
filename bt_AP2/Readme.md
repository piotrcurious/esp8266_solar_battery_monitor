Coded by chatGPT gpt4o.
It compiles with Arduino, AsyncUDP library (ESPAsyncUDP)
ESP32 board version tested is 2.0.6 (last working on 32bit linux). 

Grey hair saving tips :

-Wifi/bt coexistence works only for small packet rates. 

-If you (re)compile with different board version (f.e. 2.0.17 or 3.0.3) you need to erase entire flash, otherwise wifi will not work.

-3.0.3 coexistence is bit better but binary is larger 

-sketch is over 1.3M so you need no OTA partition scheme

-some android phones (f.e.galaxy s2) hog the Bluetooth, locking the coexistence. In such case wifi will stop working once bt is connected. I have no fix for that so far.

-3.x esp32 board has different serial definitions. You need to comment out the #define Serial0 Serial needed for 2.x
