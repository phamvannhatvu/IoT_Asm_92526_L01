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
    bool storeWateringPackage(uint32_t timestamp, float initialHumidity, float finalHumidity, float waterUsed);
    String loadLatestData();
    bool clearStorage();
    void printStoredData();
    String readStoredData();
    size_t getStoredDataCount();
    void printStorageStats();

    static constexpr size_t MAX_SENSOR_RECORDS = 1000;  // Maximum number of sensor records
    static constexpr size_t MAX_WATERING_RECORDS = 100; // Maximum number of watering records
    static constexpr size_t MAX_PACKAGES = 100; // Maximum number of watering packages

private:
    static constexpr size_t JSON_SIZE = 8192;
    const char* filename;
    bool initialized;
    void rotateArray(JsonArray& array, size_t maxSize);
};

#endif