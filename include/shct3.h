#ifndef SHCT3_H
#define SHCT3_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "modbus_rtu_master.h"

#define REGISTER_ADDRESS_HUMIDITY 0x0000
#define REGISTER_ADDRESS_TEMPERATURE 0x0001
#define REGISTER_ADDRESS_SLAVE_ID 0x07D0
#define REGISTER_ADDRESS_BAUDRATE 0x07D1

#define VALUE_BAUDRATE_2400U 0x0000
#define VALUE_BAUDRATE_4800U 0x0001
#define VALUE_BAUDRATE_9600U 0x0002

#define DEFAULT_SLAVE_ID 1
#define DEFAULT_BAUDRATE 4800U

class SHCT3 {
    private:
        ModbusMaster modbus;
        uint8_t slaveID;
        uint32_t baudrate;
        HardwareSerial* serialPort;
        uint16_t humidityValue;
        uint16_t temperatureValue;

    public:
        SHCT3();
        SHCT3(HardwareSerial* serialPort);
        SHCT3(HardwareSerial* serialPort, uint8_t slaveID, uint32_t baudrate);
        
        void setSlaveID(uint8_t slaveID);
        void setBaudrate(uint32_t baudrate);
        
        uint16_t getHumidity();
        uint16_t getTemperature();
};

#endif // SHCT3_H