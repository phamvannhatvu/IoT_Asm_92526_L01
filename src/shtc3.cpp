#include "shtc3.h"

// SHTC3::SHTC3() {
//     this->modbus = ModbusMaster();
//     this->modbus.begin(DEFAULT_BAUDRATE, DI, RO);

//     this->slaveID = DEFAULT_SLAVE_ID;
//     this->baudrate = DEFAULT_BAUDRATE;
//     this->serialPort = nullptr;
//     this->humidityValue = 0;
//     this->temperatureValue = 0;
// }

SHTC3::SHTC3(HardwareSerial* serialPort, uint8_t DI, uint8_t RO, uint8_t DE, uint8_t RE) {
    this->modbus = ModbusMaster(serialPort, DE, RE);
    this->modbus.begin(DEFAULT_BAUDRATE, DI, RO);
    this->modbus.setTimeout(1000);

    this->slaveID = DEFAULT_SLAVE_ID;
    this->baudrate = DEFAULT_BAUDRATE;
    this->serialPort = serialPort;
    this->humidityValue = 0;
    this->temperatureValue = 0;
}

SHTC3::SHTC3(HardwareSerial* serialPort, uint8_t DI, uint8_t RO, uint8_t DE, uint8_t RE, uint8_t slaveID, uint32_t baudrate) {
    this->modbus = ModbusMaster(serialPort, DE, RE);
    this->modbus.begin(DEFAULT_BAUDRATE, DI, RO);
    this->modbus.setTimeout(1000);

    switch (baudrate) {
        case 2400U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_2400U);
            this->modbus.begin(2400U, DI, RO);
            break;
        case 4800U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_4800U);
            this->modbus.begin(4800U, DI, RO);
            break;
        case 9600U:
            this->modbus.writeSingleRegister(DEFAULT_SLAVE_ID, REGISTER_ADDRESS_BAUDRATE, VALUE_BAUDRATE_9600U);
            this->modbus.begin(9600U, DI, RO);
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

void SHTC3::setSlaveID(uint8_t slaveID) {
    this->slaveID = slaveID;

}

float SHTC3::getHumidity(bool &status) {
    uint16_t result[1];
    status = this->modbus.readHoldingRegisters(this->slaveID, REGISTER_ADDRESS_HUMIDITY, 1, result);
    
    if (status == MODBUS_OK) {
        this->humidityValue = result[0] / 10.0f; // Combine high and low bytes
    } else if (status == MODBUS_ERROR_SLAVE_FAIL) {
        Serial.println("Error: Slave device not responding.");
    } else if (status == MODBUS_ERROR_ILLEGAL_FC) {
        Serial.println("Error: Illegal function code.");
    } else if (status == MODBUS_ERROR_ILLEGAL_ADDR) {
        Serial.println("Error: Illegal address.");
    } else if (status == MODBUS_ERROR_ILLEGAL_DATA) {
        Serial.println("Error: Illegal data."); 
    } else if (status == MODBUS_ERROR_TIMEOUT) {
        Serial.println("Error: Timeout while reading humidity.");
    } else if (status == MODBUS_ERROR_CRC) {
        Serial.println("Error: CRC error while reading humidity.");
    } else {
        Serial.println("Error: Unknown error while reading humidity.");
    }
    
    return this->humidityValue;
}

float SHTC3::getTemperature(bool &status) {
    uint16_t result[1];
    status = this->modbus.readHoldingRegisters(this->slaveID, REGISTER_ADDRESS_TEMPERATURE, 1, result);
    
    if (status == MODBUS_OK) {
        int16_t tempRaw = static_cast<int16_t>(result[0]);
        this->temperatureValue = static_cast<float>(tempRaw) / 10.0f;
    } else if (status == MODBUS_ERROR_SLAVE_FAIL) {
        Serial.println("Error: Slave device not responding.");
    } else if (status == MODBUS_ERROR_ILLEGAL_FC) {
        Serial.println("Error: Illegal function code.");
    } else if (status == MODBUS_ERROR_ILLEGAL_ADDR) {
        Serial.println("Error: Illegal address.");
    } else if (status == MODBUS_ERROR_ILLEGAL_DATA) {
        Serial.println("Error: Illegal data."); 
    } else if (status == MODBUS_ERROR_TIMEOUT) {
        Serial.println("Error: Timeout while reading temperature.");
    } else if (status == MODBUS_ERROR_CRC) {
        Serial.println("Error: CRC error while reading temperature.");
    } else {
        Serial.println("Error: Unknown error while reading temperature.");
    }
    
    return this->temperatureValue;
}