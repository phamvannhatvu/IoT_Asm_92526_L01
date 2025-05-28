#include "shct3.h"

SHCT3::SHCT3() {
    this->modbus = ModbusMaster();
    this->modbus.begin(DEFAULT_BAUDRATE);

    this->slaveID = DEFAULT_SLAVE_ID;
    this->baudrate = DEFAULT_BAUDRATE;
    this->serialPort = nullptr;
    this->humidityValue = 0;
    this->temperatureValue = 0;}

SHCT3::SHCT3(HardwareSerial* serialPort) {
    this->modbus = ModbusMaster(serialPort);
    this->modbus.begin(DEFAULT_BAUDRATE);

    this->slaveID = DEFAULT_SLAVE_ID;
    this->baudrate = DEFAULT_BAUDRATE;
    this->serialPort = serialPort;
    this->humidityValue = 0;
    this->temperatureValue = 0;
}

SHCT3::SHCT3(HardwareSerial* serialPort, uint8_t slaveID, uint32_t baudrate) {
    this->modbus = ModbusMaster(serialPort);
    this->modbus.begin(DEFAULT_BAUDRATE);

    switch (baudrate) {
        case 2400U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_2400U);
            this->modbus.begin(2400U);
            break;
        case 4800U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_4800U);
            this->modbus.begin(4800U);
            break;
        case 9600U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_9600U);
            this->modbus.begin(9600U);
            break;
        default:
            exit(1); // Invalid baudrate
            break;
    }
 
    this->slaveID = slaveID;
    this->baudrate = baudrate;
    this->serialPort = serialPort;
    this->humidityValue = 0;
    this->temperatureValue = 0;
}

void SHCT3::setSlaveID(uint8_t slaveID) {
    this->slaveID = slaveID;

}