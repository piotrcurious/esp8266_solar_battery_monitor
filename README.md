# esp8266_solar_battery_monitor
System to monitor solar battery using broadcast packets 
Written by hand by me , perhaps it's why there are so many bugs :D
Also added loads of code written by copilot, esp. in the junkbox folders so it's easier to jumpstart with gauges, ammeters, whatever needed. 

edit: i decided to throw changed versions and fixes into separate dirs, so pay attention to check which files are most recent to get most bug free experience. 
I use more and more AI to help me code it up, learning it progress, but I am not good coder and i have little time for it so it's slow and quirky process. 

The system is simple :
-AP (can be either one of esp's or real AP, but isolated AP is recommended, code for ESP32 and esp8266 included)
esp8266 defers packets so update rate is choppy. Use at least ESP32 as an AP if you want UDP packets being rebroadcasted instantly. 

-sender for esp8266 - connected to solar system battery
-reciever for esp8266 or esp32 - to display voltage 
-TODO: more recievers to perform actions depending on battery voltage - f.e. send IR codes to turn things like AC or TV on or off . See gauges junkbox folder. 

Idea is that there is one battery sensor, and all recievers able to get broadcast packages from it will get voltage inside struct updated. 
Those recievers can perform various functions f.e. turn on generator if battery is too low, turn on heating if battery is too high, 
etc. 
Telemetry from battery includes only voltage now, but it can include more parameters like temperature, charging current etc. 

It is not terribly reliable system. but assuming there are failsafe systems it is better than having many wires around house, 
camper van, mobile e-bike charging trailer etc. 

Cons : - esp's consume over 80mA of power.
I tried to include some power saving measures, like decrease beacon interval and make esp's use modem sleep, 
but that still consumes over 20mA. 
In worst case scenario : AP,sender,reciever for display, reciever for doing something 
this means 4 devices, 80mA each, 320mA , 1.6W (!)and probably over 2W including loss in DC-DC to power this stuff. 
I tried to use cheap DC-DC board to power bare esp8266 directly from 12V , so 12V DC-DC right down to 3.3V 
and while surprising side effect is that wifi range and packet loss improved significantly,
it did cut power consumption in idle down to 17mA only and during transmit to 80mA only. That's still too much.

One of basic uses would be to make the device control charge of the powerbank it is connected to , 
so it charges the power bank only when battery is fully charged (full sun, 13.8V)
and disconnects the charging when sun is down (battery discharging, below 13V) 

more advanced inlcludes connecting to different AP, providing internet and downloading weather forecast, 
then including heuristics for f.e. when to turn on or off the fridge or what temperature should it be set to.
f.e. simplest is - if there is full sun - set the lowest temperature possible and run the fridge, 
if sun is down and forecast says there will be no sun tommorow - set the highest temperature safe for food. 
etc. etc. 

Hint for cheap power supply for esp - if it is 12 or 24V system, car lighter usb adapter . 
Note you need to measure it's idle power before buying , as some adapters are not really brilliant. 
OTOH many actually consume below 5mA and have 90% efficiency. so why not. 

TODO : couple with better ADC as built-in ADC is horrible. 
TODO : timestamps . so far i work on creating some reliable time synchronization across nodes , see 
https://github.com/piotrcurious/esp_UDP_broadcast_time_synchronizer - created by AI , but never tested yet. 

