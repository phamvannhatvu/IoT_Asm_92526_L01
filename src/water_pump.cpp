#include "water_pump.h"

void WaterPump::begin(uint8_t signalPin) {
    this->signalPin = signalPin;
}

void WaterPump::pump(uint8_t level) {
    analogWrite(signalPin, level);
}