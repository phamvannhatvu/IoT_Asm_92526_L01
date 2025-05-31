#include "watering.h"

WateringSystem::flowRateControl(float soilHumidity, 
                                 float airHumidity, 
                                 float airTemperature, 
                                 float lightIntensity) {
    this->soilHumidityCoefficient = soilHumidity - this->soilHumidity;
    this->airHumidityCoefficient = airHumidity - this->airHumidity;
    this->airTemperatureCoefficient = airTemperature - this->airTemperature;
    this->lightIntensityCoefficient = lightIntensity - this->lightIntensity;

    // miss constraint for flow rate
    this->flowRate = soilHumidityCoefficient * this->soilHumidityTarget +
                     airHumidityCoefficient * this->airHumidityTarget +
                     airTemperatureCoefficient * this->airTemperatureTarget +
                     lightIntensityCoefficient * this->lightIntensityTarget;
}

WateringSystem::watering(float soilHumidity, 
                         float airHumidity, 
                         float airTemperature, 
                         float lightIntensity) {
    this->flowRateControl();
    this->soilHumidity = soilHumidity;
    this->airHumidity = airHumidity;
    this->airTemperature = airTemperature;
    this->lightIntensity = lightIntensity;

    if (this->soilHumidity < this->soilHumidityTarget) {
        this->wateringState = true;
        this->pumpOn(this->flowRate);
        this->flowRateControl();
    } else {
        this->wateringState = false;
        this->pumpOff();
        return;
    }
}