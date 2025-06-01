#ifndef LIGHT_H
#define LIGHT_H

#include "Arduino.h"

class LightSensor {
private:
    uint8_t signalPin;
public:
    void begin(uint8_t signalPin);
    uint32_t getBrightness();
};

#endif