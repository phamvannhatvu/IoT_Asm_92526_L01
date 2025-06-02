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
        float soilHumidityTarget;
        float soilHumidity;
        
        bool wateringState;
        float flowRate;
        
        // PI control coefficients
        uint32_t soilHumidityCoefficient_P;
        uint32_t soilHumidityCoefficient_I;
        float soilHumidityCoefficient_Sum;

        float waterUsed;
        unsigned long lastWateringTime;

        void pumpOn(float flowRate);
        void pumpOff();
        void flowRateControl(float soilHumidity);

    public:
        WateringSystem(float soilHumidityTarget);

        void watering(float soilHumidity);
        void setWateringState(bool state);
        bool isWatering();
        float getFlowRate();
        float getWaterUsed() const { return waterUsed; }
        void resetWaterUsed() { waterUsed = 0.0f; }
};

#endif // WATERING_H