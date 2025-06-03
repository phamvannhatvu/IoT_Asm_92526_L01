#include "watering.h"

WateringSystem::WateringSystem(float soilHumidityTarget) {
    this->soilHumidityTarget = soilHumidityTarget;
    
    // Initialize coefficients
    this->soilHumidityCoefficient_P = 1;
    this->soilHumidityCoefficient_I = 1;
    this->soilHumidityCoefficient_Sum = 0;
    
    // Initialize flow rate tracking
    this->startTime = 0;
    this->totalFlowRate = 0;
    this->flowRateCount = 0;
    this->wateringState = false;
    this->flowRate = 0;

    this->pumpOff();
}

void WateringSystem::flowRateControl(float soilHumidity) {
    this->soilHumidityCoefficient_Sum += (this->soilHumidityTarget - soilHumidity);
}

void WateringSystem::flowRateControl(float soilHumidity, 
                                 float airHumidity, 
                                 float airTemperature, 
                                 float lightIntensity) {
    this->soilHumidityCoefficient_Sum += (this->soilHumidityTarget - BRAKE_COEFFICIENT - soilHumidity);
    // this->airHumidityCoefficient = airHumidity - this->airHumidity;
    // this->airTemperatureCoefficient = airTemperature - this->airTemperature;
    // this->lightIntensityCoefficient = lightIntensity - this->lightIntensity;

    this->flowRate = (this->soilHumidityTarget - soilHumidity) * this->soilHumidityCoefficient_P
                    + this->soilHumidityCoefficient_Sum * this->soilHumidityCoefficient_I;
    if (this->flowRate > PUMP_POWER_MAX) {
        this->flowRate = PUMP_POWER_MAX;
    } else if (this->flowRate < PUMP_POWER_MIN) {
        this->flowRate = PUMP_POWER_MIN;
    }
}

void WateringSystem::watering(float soilHumidity) {
    this->flowRateControl(soilHumidity);
    this->soilHumidity = soilHumidity;

    if (this->soilHumidity < this->soilHumidityTarget) {
        if (!this->wateringState) {
            // Start new watering cycle
            this->startTime = millis();
            this->totalFlowRate = 0;
            this->flowRateCount = 0;
        }
        
        this->wateringState = true;
        this->pumpOn(this->flowRate);
        
        // Track flow rate
        this->totalFlowRate += this->flowRate;
        this->flowRateCount++;
    } else {
        this->wateringState = false;
        this->soilHumidityCoefficient_Sum = 0;
        this->pumpOff();
    }
}

float WateringSystem::getAverageFlowRate() const {
    return flowRateCount > 0 ? totalFlowRate / flowRateCount : 0;
}

unsigned long WateringSystem::getWateringDuration() const {
    return wateringState ? (millis() - startTime) : (startTime > 0 ? millis() - startTime : 0);
}

void WateringSystem::pumpOn(float flowRate) {
    // Simulate pump on with the given flow rate
    Serial.print("Pump is ON with flow rate: ");
    Serial.println(flowRate);
    analogWrite(PUMP_PIN, flowRate);
}

void WateringSystem::pumpOff() {
    // Simulate pump off
    Serial.println("Pump is OFF");
    analogWrite(PUMP_PIN, 0);
    float avgFlowRate = this->getAverageFlowRate();
    Serial.printf("Average flow rate: %.2f ml/s\n", avgFlowRate);
}