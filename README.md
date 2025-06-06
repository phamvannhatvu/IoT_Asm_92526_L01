# IoT Assignment  
**Group:** 92526  
**Class:** L01  

## Project Overview  
This repository branch is dedicated to the development of an **ESP-NOW Sensor Node**. The sensor node is designed to read current environmental conditions and control actuators, such as LEDs for lighting and a water pump for soil moisture management.

## Functionality  
The system provides the following features:
- Monitors ambient light levels and adjusts lighting via LEDs.
- Measures soil moisture levels and activates a water pump as needed.
- Records water pump activity history.
- Reads air temperature and humidity.
- Communicates with other modules using Modbus RTU protocol over ESP32.

## Source Code Components

- **`data_storage.h`**, **`data_storage.cpp`**    
  Stores historical data of water pump activities.

- **`light.h`**, **`light.cpp`**  
  Reads current light levels and controls LEDs accordingly.

- **`sthc3.h`**, **`sthc3.cpp`**  
  Interfaces with the soil moisture sensor to retrieve readings.

- **`modbus_rtu_master.h`**, **`modbus_rtu_master.cpp`**  
  Implements Modbus RTU communication protocol for the master device (ESP32 Sensor Node).

- **`temp_humid.h`**, **`temp_humid.cpp`**  
  Reads data from the air temperature and humidity sensor.

- **`watering.h`**, **`watering.cpp`**  
Contains the logic and algorithms to perform automatic watering based on sensor input.

- **`main.cpp`**  
  The entry point of the application. Initializes modules and manages communication with the ESP-NOW Gateway.
