#include "watering.h"

WateringSystem::WateringSystem(float soilHumidityTarget, 
                                 float airHumidityTarget, 
                                 float airTemperatureTarget, 
                                 float lightIntensityTarget) {
    this->soilHumidityTarget = soilHumidityTarget;
    this->airHumidityTarget = airHumidityTarget;
    this->airTemperatureTarget = airTemperatureTarget;
    this->lightIntensityTarget = lightIntensityTarget;

    // Initialize coefficients
    this->soilHumidityCoefficient_P = 10;
    this->soilHumidityCoefficient_I = 10;
    this->soilHumidityCoefficient_Sum = 0;

    this->waterUsed = 0.0f;
    this->lastWateringTime = 0;
}

void WateringSystem::flowRateControl(float soilHumidity, 
                                 float airHumidity, 
                                 float airTemperature, 
                                 float lightIntensity) {
    this->soilHumidityCoefficient_Sum += (this->soilHumidityTarget- soilHumidity);
    // this->airHumidityCoefficient = airHumidity - this->airHumidity;
    // this->airTemperatureCoefficient = airTemperature - this->airTemperature;
    // this->lightIntensityCoefficient = lightIntensity - this->lightIntensity;

    this->flowRate = (this->soilHumidityTarget - soilHumidity) * this->soilHumidityCoefficient_P
                    + this->soilHumidityCoefficient_Sum * this->soilHumidityCoefficient_I;
    if (this->flowRate > 10000) {
        this->flowRate = 10000;
    } else if (this->flowRate < 0) {
        this->flowRate = 0;
    }
}

void WateringSystem::watering(float soilHumidity, 
                         float airHumidity, 
                         float airTemperature, 
                         float lightIntensity) {
    this->flowRateControl(soilHumidity, 
                          airHumidity, 
                          airTemperature, 
                          lightIntensity);
    this->soilHumidity = soilHumidity;
    this->airHumidity = airHumidity;
    this->airTemperature = airTemperature;
    this->lightIntensity = lightIntensity;

    if (this->soilHumidity < this->soilHumidityTarget) {
        this->wateringState = true;
        this->pumpOn(this->flowRate);
        
        // Calculate water used since last check
        unsigned long currentTime = millis();
        if (lastWateringTime > 0) {
            // flowRate is in ml/hour, convert to ml for the time period
            float timeHours = (currentTime - lastWateringTime) / 3600000.0f; // Convert ms to hours
            waterUsed += flowRate * timeHours;
        }
        lastWateringTime = currentTime;
    } else {
        this->wateringState = false;
        this->soilHumidityCoefficient_Sum = 0;
        this->pumpOff();
        lastWateringTime = 0; // Reset timing when pump stops
        return;
    }
}
void WateringSystem::pumpOn(float flowRate) {
    // Simulate pump on with the given flow rate
    Serial.print("Pump is ON with flow rate: ");
    Serial.println(flowRate);
}
void WateringSystem::pumpOff() {
    // Simulate pump off
    Serial.println("Pump is OFF");
    Serial.print("Total water used: ");
    Serial.print(waterUsed);
    Serial.println(" ml");
}

void WateringSystem::setWateringState(bool state) {
    this->wateringState = state;
}

bool WateringSystem::isWatering() {
    return this->wateringState;
}

float WateringSystem::getFlowRate() {
    return this->flowRate;
}