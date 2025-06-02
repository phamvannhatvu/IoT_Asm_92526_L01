#ifndef DATA_STORAGE_H
#define DATA_STORAGE_H

#include <Arduino.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

class DataStorage {
public:
    DataStorage(const char* filename = "/sensor_data.json");
    bool begin();
    bool storeSensorData(float soilHumidity, float soilTemperature);
    bool storeWateringData(float initialHumidity, float waterUsed);
    bool storeWeatherData(float temperature, float humidity, float windSpeed, float rain);
    bool storeWateringPackage(float initialHumidity, float finalHumidity, 
                             float avgFlowRate, unsigned long duration);  // Updated parameters
    String loadLatestData();
    bool clearStorage();
    void printStoredData();
    String readStoredData();
    size_t getStoredDataCount();
    void printStorageStats();
    bool printWateringPackage(size_t index);  // Add this line
    size_t getWateringPackageCount();         // Add this line
    void calculateWaterUsageStats();
    void calculateFlowRateStats();  // Add this method for flow rate statistics
    
    static constexpr size_t MAX_SENSOR_RECORDS = 1000;  // Maximum number of sensor records
    static constexpr size_t MAX_WATERING_RECORDS = 100; // Maximum number of watering records
    static constexpr size_t MAX_PACKAGES = 100; // Maximum number of watering packages
    static constexpr size_t NUM_HUMIDITY_GROUPS = 10;  // 0-10, 10-20, ..., 90-100
    static constexpr float GROUP_SIZE = 10.0f;

private:
    static constexpr size_t JSON_SIZE = 8192;
    const char* filename;
    bool initialized;
    void rotateArray(JsonArray& array, size_t maxSize);
};

#endif