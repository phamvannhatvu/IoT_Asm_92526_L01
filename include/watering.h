#ifndef WATERING_H
#define WATERING_H

#include <Arduino.h>
#include <ThingsBoard.h>
// #include <PubSubClient.h>
// #include <ArduinoJson.h>
// #include <scheduler.h>

// Pin Definitions
// #define SOIL_MOISTURE_PIN     34    // Analog pin for soil moisture sensor
// #define PUMP_RELAY_PIN       16    // Digital pin for water pump relay
// #define VALVE_RELAY_PIN      17    // Digital pin for valve relay

// Thresholds
// #define SOIL_HUMIDITY_TARGET 70.0f // Target soil humidity percentage

// Timing constants
// #define WATERING_DURATION     10000  // Duration for watering cycle (ms)
// #define SENSOR_READ_INTERVAL  5000   // Interval between sensor readings (ms)
// #define PUBLISH_INTERVAL     30000   // Interval for publishing data to ThingsBoard (ms)

class WateringSystem {
    private:
        // Sensors
        float soilHumidityTarget; // Target soil humidity level
        float airHumidityTarget;  // Target air humidity level
        float airTemperatureTarget; // Target air temperature
        float lightIntensityTarget; // Target light intensity level

        float soilHumidity;      // Current soil humidity level
        float airHumidity;       // Current air humidity level
        float airTemperature;    // Current air temperature
        float lightIntensity;    // Current light intensity
        
        // Watering System
        bool wateringState;         // Current pump state
        float waterAmout;          // Amount of water dispensed
        float flowRate;            // Flow rate of the water pump

        // Learning System
        uint32_t soilHumidityCoefficient_P;
        uint32_t soilHumidityCoefficient_I;
        float soilHumidityCoefficient_Sum;
        uint32_t airHumidityCoefficient;
        uint32_t airTemperatureCoefficient;
        uint32_t lightIntensityCoefficient;

        float waterUsed;           // Track total water used in ml
        unsigned long lastWateringTime; // Track time for water usage calculation

    

        void pumpOn(float flowRate);
        void pumpOff();

        void flowRateControl(float soilHumidity, 
                             float airHumidity, 
                             float airTemperature, 
                             float lightIntensity);
    public:
        // Constructor & Destructor
        WateringSystem(float soilHumidityTarget, 
                       float airHumidityTarget, 
                       float airTemperatureTarget, 
                       float lightIntensityTarget);
        // ~WateringSystem();

        void watering(float soilHumidity, 
                      float airHumidity, 
                      float airTemperature, 
                      float lightIntensity);

        void setWateringState(bool state);
        bool isWatering();
        float getFlowRate();
        float getWaterUsed() const { return waterUsed; }
        void resetWaterUsed() { waterUsed = 0.0f; }
};

#endif // WATERING_H