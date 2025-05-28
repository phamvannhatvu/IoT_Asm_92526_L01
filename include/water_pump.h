#ifndef WATER_PUMP_H
#define WATER_PUMP_H

#include <Arduino.h>

class WaterPump {
private:
    uint8_t signalPin;
public:
    void begin(uint8_t signalPin);
    void pump(uint8_t level);
};


#endif