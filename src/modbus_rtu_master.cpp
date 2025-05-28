#include "modbus_rtu_master.h"

ModbusMaster::ModbusMaster() {
    serial = nullptr;
    timeout = MODBUS_TIMEOUT_MS;
}

ModbusMaster::ModbusMaster(HardwareSerial* serialPort) {
    serial = serialPort;
    timeout = MODBUS_TIMEOUT_MS;
}

void ModbusMaster::begin(unsigned long baud) {
    serial->begin(baud);
}

void ModbusMaster::setTimeout(unsigned long timeoutMs) {
    timeout = timeoutMs;
}

void ModbusMaster::clearSerialBuffer() {
    while (serial->available()) {
        serial->read();
    }
}

bool ModbusMaster::waitForResponse(uint16_t expectedLength) {
    unsigned long startTime = millis();
    uint16_t bytesRead = 0;

    while ((millis() - startTime) < timeout) {
        if (serial->available()) {
            rxBuffer[bytesRead++] = serial->read();
            if (bytesRead >= expectedLength) {
                return true;
            }
        }
        delay(1);
    }
    return false;
}

uint16_t ModbusMaster::calculateCRC(uint8_t* buffer, uint16_t length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool ModbusMaster::validateResponse(uint8_t* buffer, uint16_t length) {
    uint16_t receivedCRC = (buffer[length - 1] << 8) | buffer[length - 2];
    uint16_t calculatedCRC = calculateCRC(buffer, length - 2);
    return receivedCRC == calculatedCRC;
}

uint8_t ModbusMaster::readHoldingRegisters(uint8_t slaveId, uint16_t address, 
                                         uint16_t quantity, uint16_t* result) {
    clearSerialBuffer();

    // Build request
    txBuffer[0] = slaveId;
    txBuffer[1] = MODBUS_READ_HOLDING_REG;
    txBuffer[2] = (address >> 8) & 0xFF;
    txBuffer[3] = address & 0xFF;
    txBuffer[4] = (quantity >> 8) & 0xFF;
    txBuffer[5] = quantity & 0xFF;
    
    uint16_t crc = calculateCRC(txBuffer, 6);
    txBuffer[6] = crc & 0xFF;
    txBuffer[7] = (crc >> 8) & 0xFF;

    // Send request
    serial->write(txBuffer, 8);
    
    // Calculate expected response length
    uint8_t byteCount = quantity * 2;
    uint8_t expectedLength = byteCount + 5;  // slave + FC + byteCount + Data + CRC(2)
    
    // Wait for response
    if (!waitForResponse(expectedLength)) {
        return MODBUS_ERROR_TIMEOUT;
    }
    
    if (!validateResponse(rxBuffer, expectedLength)) {
        return MODBUS_ERROR_CRC;
    }
    
    if (rxBuffer[0] != slaveId) {
        return MODBUS_ERROR_SLAVE_FAIL;
    }
    
    if (rxBuffer[1] == (MODBUS_READ_HOLDING_REG + 0x80)) {
        return rxBuffer[2]; // Exception code
    }
    
    // Extract data
    for (uint16_t i = 0; i < quantity; i++) {
        result[i] = (rxBuffer[3 + i * 2] << 8) | rxBuffer[4 + i * 2];
    }
    
    return MODBUS_OK;
}

uint8_t ModbusMaster::writeSingleRegister(uint8_t slaveId, uint16_t address, uint16_t value) {
    clearSerialBuffer();

    // Build request
    txBuffer[0] = slaveId;
    txBuffer[1] = MODBUS_WRITE_REG;
    txBuffer[2] = (address >> 8) & 0xFF;
    txBuffer[3] = address & 0xFF;
    txBuffer[4] = (value >> 8) & 0xFF;
    txBuffer[5] = value & 0xFF;
    
    uint16_t crc = calculateCRC(txBuffer, 6);
    txBuffer[6] = crc & 0xFF;
    txBuffer[7] = (crc >> 8) & 0xFF;

    serial->write(txBuffer, 8);
    
    if (!waitForResponse(8)) {
        return MODBUS_ERROR_TIMEOUT;
    }
    
    if (!validateResponse(rxBuffer, 8)) {
        return MODBUS_ERROR_CRC;
    }
    
    if (rxBuffer[0] != slaveId) {
        return MODBUS_ERROR_SLAVE_FAIL;
    }
    
    return MODBUS_OK;
}

uint8_t ModbusMaster::pingSlaveDevice(uint8_t slaveId) {
    return readHoldingRegisters(slaveId, 0, 1, nullptr);
}

bool ModbusMaster::isSlaveAvailable(uint8_t slaveId) {
    return (pingSlaveDevice(slaveId) == MODBUS_OK);
}