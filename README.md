## ESP-NOW Gateway  
**Group:** 92526  
**Class:** L01  

### Project Overview  
This branch is dedicated to the development of the **ESP-NOW Gateway**, which acts as a communication hub between the **ESP-NOW Sensor Node modules** and the **CoreIoT** platform. It handles telemetry transmission, RPC subscriptions, and integrates weather data to support smarter decision-making (e.g., adaptive watering).

### Responsibilities  
- Communicates with **CoreIoT** to:
  - Send telemetry data
  - Subscribe to Remote Procedure Calls (RPCs)
- Interfaces with ESP-NOW Sensor Node modules
- Retrieves real-time weather information to optimize watering logic

### Source Code Components

- **`main.cpp`**  
  The main entry point. Coordinates data flow and system behavior.

- **`weather.h`, `weather.cpp`**  
  Implements weather data retrieval using the OpenWeatherMap API.
