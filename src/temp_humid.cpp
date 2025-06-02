#include "temp_humid.h"
#include "Wire.h"

void TempHumidSensor::begin() {
    Wire.begin();
    dht20.begin();
}

void TempHumidSensor::get_value(float &temp, float &humid) {
    dht20.read();
    temp = dht20.getTemperature();
    humid = dht20.getHumidity();          
}