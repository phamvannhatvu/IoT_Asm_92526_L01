#include "light.h"

void LightSensor::begin(uint8_t signalPin) {
    this->signalPin = signalPin;
}

uint32_t LightSensor::getBrightness() {
    return analogRead(signalPin) / 4095.0 * 70000;
}