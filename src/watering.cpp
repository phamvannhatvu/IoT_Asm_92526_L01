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
    this->soilHumidityCoefficient_P = 1;
    this->soilHumidityCoefficient_I = 1;
    this->soilHumidityCoefficient_Sum = 0;

    this->pumpOff();
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
    } else {
        this->wateringState = false;
        this->soilHumidityCoefficient_Sum = 0; // Reset the sum for the next cycle
        this->pumpOff();
    }
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