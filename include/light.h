#ifndef LIGHT_H
#define LIGHT_H

#include "Arduino.h"

#define LIGHT_OFF_THRESHOLD 2048

class LightSensor {
private:
    uint8_t signalPin;
    uint8_t ledPin;
public:
    void begin(uint8_t signalPin, uint8_t ledPin);
    void controlLED();
};

#endif