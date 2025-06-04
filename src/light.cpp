#include "light.h"

void LightSensor::begin(uint8_t signalPin, uint8_t ledPin) {
    this->signalPin = signalPin;
    this->ledPin = ledPin;

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
}

void LightSensor::controlLED() {
    Serial.print("Brightness: ");
    Serial.println(analogRead(signalPin));
    if (analogRead(signalPin) < LIGHT_OFF_THRESHOLD) {
        digitalWrite(ledPin, HIGH);
    } else {
        digitalWrite(ledPin, LOW);
    }
}