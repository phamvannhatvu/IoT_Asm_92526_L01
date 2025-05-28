#ifndef MODBUS_RTU_MASTER_H
#define MODBUS_RTU_MASTER_H

#include <Arduino.h>
#include <HardwareSerial.h>

// MODBUS function codes
#define MODBUS_READ_COILS          0x01
#define MODBUS_READ_DISCRETE       0x02
#define MODBUS_READ_HOLDING_REG    0x03
#define MODBUS_READ_INPUT_REG      0x04
#define MODBUS_WRITE_COIL         0x05
#define MODBUS_WRITE_REG          0x06
#define MODBUS_WRITE_COILS        0x0F
#define MODBUS_WRITE_REGS         0x10

// Error codes
#define MODBUS_OK                  0x00
#define MODBUS_ERROR_ILLEGAL_FC    0x01
#define MODBUS_ERROR_ILLEGAL_ADDR  0x02
#define MODBUS_ERROR_ILLEGAL_DATA  0x03
#define MODBUS_ERROR_SLAVE_FAIL    0x04
#define MODBUS_ERROR_ACK           0x05
#define MODBUS_ERROR_BUSY          0x06
#define MODBUS_ERROR_CRC           0x07
#define MODBUS_ERROR_TIMEOUT       0x08

// Configuration
#define MODBUS_MAX_BUFFER         256
#define MODBUS_TIMEOUT_MS         1000
#define MODBUS_INTERFRAME_DELAY   50

class ModbusMaster {
    private:
        HardwareSerial* serial;
        uint8_t rxBuffer[MODBUS_MAX_BUFFER];
        uint8_t txBuffer[MODBUS_MAX_BUFFER];
        uint32_t timeout;
        
        // Utility functions
        uint16_t calculateCRC(uint8_t* buffer, uint16_t length);
        bool validateResponse(uint8_t* buffer, uint16_t length);
        void clearSerialBuffer();
        bool waitForResponse(uint16_t expectedLength);
        
    public:
        ModbusMaster();
        ModbusMaster(HardwareSerial* serialPort);
        
        // Configuration
        void begin(uint32_t baud, uint16_t TX, uint16_t RX);
        void setTimeout(uint32_t timeoutMs);
        
        // Read functions
        uint8_t readHoldingRegisters(uint8_t slaveId, uint16_t address, 
                                   uint16_t quantity, uint16_t* result);
        uint8_t readInputRegisters(uint8_t slaveId, uint16_t address, 
                                 uint16_t quantity, uint16_t* result);
        uint8_t readCoils(uint8_t slaveId, uint16_t address, 
                         uint16_t quantity, uint8_t* result);
        uint8_t readDiscreteInputs(uint8_t slaveId, uint16_t address, 
                                 uint16_t quantity, uint8_t* result);
        
        // Write functions
        uint8_t writeSingleRegister(uint8_t slaveId, uint16_t address, 
                                  uint16_t value);
        uint8_t writeSingleCoil(uint8_t slaveId, uint16_t address, 
                              bool value);
        uint8_t writeMultipleRegisters(uint8_t slaveId, uint16_t address, 
                                     uint16_t quantity, uint16_t* values);
        uint8_t writeMultipleCoils(uint8_t slaveId, uint16_t address, 
                                 uint16_t quantity, uint8_t* values);
        
        // Diagnostic functions
        uint8_t pingSlaveDevice(uint8_t slaveId);
        bool isSlaveAvailable(uint8_t slaveId);
};

#endif // MODBUS_RTU_MASTER_H