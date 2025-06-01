#include "data_storage.h"

DataStorage::DataStorage(const char* filename) : filename(filename), initialized(false) {}

bool DataStorage::begin() {
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        return false;
    }
    initialized = true;
    return true;
}

void DataStorage::rotateArray(JsonArray& array, size_t maxSize) {
    if (array.size() >= maxSize) {
        // Remove oldest entry (first element)
        array.remove(0);
    }
}

bool DataStorage::storeSensorData(float soilHumidity, float soilTemperature) {
    if (!initialized) return false;

    StaticJsonDocument<JSON_SIZE> doc;
    
    // Load existing data
    File file = SPIFFS.open(filename, "r");
    if (file) {
        DeserializationError error = deserializeJson(doc, file);
        file.close();
        if (error) {
            doc.clear();
        }
    }

    // Create sensor_data array if it doesn't exist
    if (!doc.containsKey("sensor_data")) {
        doc.createNestedArray("sensor_data");
    }

    // Add new data
    JsonArray sensorArray = doc["sensor_data"].as<JsonArray>();
    JsonObject sensorData = sensorArray.createNestedObject();
    sensorData["timestamp"] = millis();
    sensorData["soil_humidity"] = soilHumidity;
    sensorData["soil_temperature"] = soilTemperature;

    // Rotate array if needed
    rotateArray(sensorArray, MAX_SENSOR_RECORDS);

    // Save to file
    file = SPIFFS.open(filename, "w");
    if (!file) return false;
    
    serializeJson(doc, file);
    file.close();
    return true;
}

bool DataStorage::storeWateringData(float initialHumidity, float waterUsed) {
    if (!initialized) return false;

    StaticJsonDocument<JSON_SIZE> doc;
    
    File file = SPIFFS.open(filename, "r");
    if (file) {
        DeserializationError error = deserializeJson(doc, file);
        file.close();
        if (error) {
            doc.clear();
        }
    }

    // Create watering_data array if it doesn't exist
    if (!doc.containsKey("watering_data")) {
        doc.createNestedArray("watering_data");
    }

    JsonArray wateringArray = doc["watering_data"].as<JsonArray>();
    JsonObject wateringData = wateringArray.createNestedObject();
    wateringData["timestamp"] = millis();
    wateringData["initial_humidity"] = initialHumidity;
    wateringData["water_used"] = waterUsed;
    wateringData["date"] = time(nullptr); // Add actual date/time

    // Rotate array if needed
    rotateArray(wateringArray, MAX_WATERING_RECORDS);

    file = SPIFFS.open(filename, "w");
    if (!file) return false;
    
    serializeJson(doc, file);
    file.close();
    return true;
}

bool DataStorage::storeWateringPackage(uint32_t timestamp, float initialHumidity, float finalHumidity, float waterUsed) {
    if (!initialized) return false;

    StaticJsonDocument<JSON_SIZE> doc;
    
    // Load existing data
    File file = SPIFFS.open(filename, "r");
    if (file) {
        DeserializationError error = deserializeJson(doc, file);
        file.close();
        if (error) {
            doc.clear();
        }
    }

    // Create watering_packages array if it doesn't exist
    if (!doc.containsKey("watering_packages")) {
        doc.createNestedArray("watering_packages");
    }

    JsonArray packages = doc["watering_packages"].as<JsonArray>();
    
    // Remove oldest package if we reached the limit
    if (packages.size() >= MAX_PACKAGES) {
        packages.remove(0);
    }

    // Add new package
    JsonObject package = packages.createNestedObject();
    package["timestamp"] = timestamp;
    package["initial_humidity"] = initialHumidity;
    package["final_humidity"] = finalHumidity;
    package["water_used"] = waterUsed;

    // Save to file
    file = SPIFFS.open(filename, "w");
    if (!file) return false;
    
    serializeJson(doc, file);
    file.close();
    return true;
}

String DataStorage::loadLatestData() {
    if (!initialized) return "{}";

    File file = SPIFFS.open(filename, "r");
    if (!file) return "{}";

    String data = file.readString();
    file.close();
    return data;
}

bool DataStorage::clearStorage() {
    if (!initialized) return false;
    return SPIFFS.remove(filename);
}

void DataStorage::printStoredData() {
    if (!initialized) {
        Serial.println("Storage not initialized!");
        return;
    }

    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.println("\n--- Stored Data ---");
    while (file.available()) {
        String line = file.readStringUntil('\n');
        Serial.println(line);
    }
    Serial.println("--- End of Data ---\n");
    file.close();
}

String DataStorage::readStoredData() {
    if (!initialized) return "Storage not initialized";
    
    File file = SPIFFS.open(filename, "r");
    if (!file) return "Failed to open file";
    
    String content = file.readString();
    file.close();
    return content;
}

size_t DataStorage::getStoredDataCount() {
    if (!initialized) return 0;
    
    StaticJsonDocument<JSON_SIZE> doc;
    File file = SPIFFS.open(filename, "r");
    if (!file) return 0;
    
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) return 0;
    
    size_t sensorCount = doc["sensor_data"].size();
    size_t wateringCount = doc["watering_data"].size();
    
    return sensorCount + wateringCount;
}

void DataStorage::printStorageStats() {
    if (!initialized) {
        Serial.println("Storage not initialized!");
        return;
    }

    StaticJsonDocument<JSON_SIZE> doc;
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Failed to open storage file!");
        return;
    }

    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("Failed to parse storage data!");
        return;
    }

    size_t packageCount = doc["watering_packages"].size();

    Serial.println("\nStorage Statistics:");
    Serial.printf("Watering Packages: %d/%d (%.1f%% full)\n", 
        packageCount, MAX_PACKAGES,
        (float)packageCount / MAX_PACKAGES * 100);
    Serial.printf("SPIFFS Usage: %d/%d bytes (%.1f%% used)\n",
        SPIFFS.usedBytes(), SPIFFS.totalBytes(),
        (float)SPIFFS.usedBytes() / SPIFFS.totalBytes() * 100);
}