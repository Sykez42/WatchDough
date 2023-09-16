# WatchDough
Smart sourdough incubator powered by STM32 IoT node

## Softwares and tools
- Node-RED
- STM32 CubeIde

## Project requirements

|REQ ID|TYPE|DESCRIPTION|
|---|---|---|
|RQ001|HL|the system must read temperature (T) and humidity (H) values of the incubator|
|RQ002|HL|the system must turn on and off an IR lamp inside the incubator|
|RQ003|HL|the system must interface with an MQTT server through a mobile application|
|RQ004|HL|the system must send humidity and temperature data to the MQTT server|
|RQ005|HL|the system must have two modes: AUTO and MANUAL|
|RQ006|ML|in both modes, H and T values must be shown in the application interface|
|RQ007|ML|in MANUAL mode, the IR lamp is turned on and off  through an MQTT command|
|RQ008|HL|in AUTO mode, the system must receive humidity and temperature set points (T_st and H_st) from the MQTT application|
|RQ009|HL|in AUTO mode, the system must receive the humidity variability percentage (H%) from the MQTT server|
|RQ010|ML|in AUTO mode, the code must compare the set points values with values read from the HT sensor|
|RQ011|ML|in AUTO mode, if T < T_st the code must turn the IR lamp on|
|RQ012|ML|in AUTO mode, if T >= T_st the code must turn the IR lamp off|
|RQ013|ML|in AUTO mode, if H < H_st - H% or if H > H_st + H% the code sends an alarm through the MQTT server |
|RQ014|ML|in both modes, the lamp status (ON-OFF) must be shown on the MQTT application interface|
|RQ015|LL|the IR lamp is controlled by a digital relay|

## Useful links
- [Sensor usage tutorial](https://wiki.st.com/stm32mcu/wiki/STM32StepByStep:Step4_Sensors_usage)




