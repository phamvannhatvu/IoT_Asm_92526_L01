#ifndef WATERING_H
#define WATERING_H

#include <Arduino.h>
#include <ThingsBoard.h>
#include "data_storage.h"

#define PUMP_PIN D9
#define PUMP_POWER_MIN 50
#define PUMP_POWER_MAX 255
#define BRAKE_COEFFICIENT 10
#define WATERING_SAMPLE 3

class WateringSystem {
    private:
        float soilHumidityTarget;
        float soilHumidity;
        
        bool wateringState;
        float flowRate;
        
        // Flow rate tracking
        unsigned long startTime;
        float totalFlowRate;
        int flowRateCount;
        
        // PI control coefficients
        uint32_t soilHumidityCoefficient_P;
        uint32_t soilHumidityCoefficient_I;
        float soilHumidityCoefficient_Sum;

        // Time-based watering control
        bool usingHistoricalControl;
        float targetWateringTime;
        
        bool wateringMode = false; // false: automatic, true: manual

        void pumpOn(float flowRate);
        void pumpOff();
        void flowRateControl(float soilHumidity, HumidityGroupStats stats, float& expectedTime);

    public:
        WateringSystem(float soilHumidityTarget);
        void watering(float soilHumidity, HumidityGroupStats stats);
        bool isWatering() const { return wateringState; }
        bool isUsingHistoricalControl() const { return usingHistoricalControl; }
        float getFlowRate() const { return flowRate; }
        float getAverageFlowRate() const;
        unsigned long getWateringDuration() const;
};

#endif // WATERING_H