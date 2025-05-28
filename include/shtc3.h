#ifndef SHTC3_H
#define SHTC3_H

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

class SHTC3 {
    private:
        ModbusMaster modbus;
        uint8_t slaveID;
        uint32_t baudrate;
        HardwareSerial* serialPort;
        float humidityValue;
        float temperatureValue;

    public:
        // SHTC3();
        SHTC3(HardwareSerial* serialPort, uint16_t TX, uint16_t RX);
        SHTC3(HardwareSerial* serialPort, uint16_t TX, uint16_t RX, uint8_t slaveID, uint32_t baudrate);
        
        void setSlaveID(uint8_t slaveID);
        void setBaudrate(uint32_t baudrate);
        
        float getHumidity();
        float getTemperature();
};

#endif // SHTC3_H