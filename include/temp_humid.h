#ifndef TEMP_HUMID_H
#define TEMP_HUMID_H

#include <Arduino.h>
#include "DHT20.h"

class TempHumidSensor {
private:
    DHT20 dht20;
public:
    void begin();
    void get_value(float &temp, float &humid);
};

#endif