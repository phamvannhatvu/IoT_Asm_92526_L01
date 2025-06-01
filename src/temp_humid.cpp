#include "temp_humid.h"
#include "Wire.h"

void TempHumidSensor::begin() {
    Wire.begin();
    dht20.begin();
}

void TempHumidSensor::get_value(float &temp, float &humid) {
    // dht20.read();
    temp = analogRead(36) / 4095.0 * 50;// dht20.getTemperature();
    humid = analogRead(36) / 4095.0 * 100;// dht20.getHumidity();          
}