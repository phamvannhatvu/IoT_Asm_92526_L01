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
    this->soilHumidityCoefficient = 1; // Default value, can be adjusted
}

void WateringSystem::flowRateControl(float soilHumidity, 
                                 float airHumidity, 
                                 float airTemperature, 
                                 float lightIntensity) {
    this->soilHumidityCoefficient = soilHumidity - this->soilHumidity;
    // this->airHumidityCoefficient = airHumidity - this->airHumidity;
    // this->airTemperatureCoefficient = airTemperature - this->airTemperature;
    // this->lightIntensityCoefficient = lightIntensity - this->lightIntensity;

    // miss constraint for flow rate
    this->flowRate = soilHumidityCoefficient * this->soilHumidityTarget;// +
                    //  airHumidityCoefficient * this->airHumidityTarget +
                    //  airTemperatureCoefficient * this->airTemperatureTarget +
                    //  lightIntensityCoefficient * this->lightIntensityTarget;
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
        this->pumpOff();
        return;
    }
}

float WateringSystem::getFlowRate() {
    return this->flowRate;
}