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

bool DataStorage::storeWateringPackage(float initialHumidity, float finalHumidity, 
                                     float avgFlowRate, unsigned long duration) {
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

    // Add new package with flow rate and duration instead of water used
    JsonObject package = packages.createNestedObject();
    package["initial_humidity"] = initialHumidity;
    package["final_humidity"] = finalHumidity;
    package["avg_flow_rate"] = avgFlowRate;     // Store flow rate in ml/s
    package["duration"] = duration;              // Store duration in milliseconds

    // Save to file
    file = SPIFFS.open(filename, "w");
    if (!file) return false;
    
    serializeJson(doc, file);
    file.close();
    return true;
}

bool DataStorage::clearStorage() {
    if (!initialized) {
        Serial.println("Storage not initialized!");
        return false;
    }

    // First try to remove the file
    if (SPIFFS.remove(filename)) {
        // Create a new empty file with initial structure
        File file = SPIFFS.open(filename, "w");
        if (!file) {
            Serial.println("Failed to create new storage file!");
            return false;
        }

        // Create empty JSON structure
        StaticJsonDocument<JSON_SIZE> doc;
        doc.createNestedArray("watering_packages");
        
        // Write empty structure to file
        if (serializeJson(doc, file) == 0) {
            Serial.println("Failed to write initial structure!");
            file.close();
            return false;
        }

        file.close();
        return true;
    }

    return false;
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

bool DataStorage::printWateringPackage(size_t index) {
    if (!initialized) {
        Serial.println("Storage not initialized!");
        return false;
    }

    StaticJsonDocument<JSON_SIZE> doc;
    File file = SPIFFS.open(filename, "r");
    if (!file) {
        Serial.println("Failed to open storage file!");
        return false;
    }

    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.println("Failed to parse storage data!");
        return false;
    }

    JsonArray packages = doc["watering_packages"].as<JsonArray>();
    if (index >= packages.size()) {
        Serial.printf("Invalid package index! Max index: %d\n", packages.size() - 1);
        return false;
    }

    JsonObject package = packages[index];
    
    Serial.println("\n=== Watering Package Data ===");
    Serial.printf("Package Index: %d\n", index);
    Serial.printf("Initial Humidity: %.1f%%\n", package["initial_humidity"].as<float>());
    Serial.printf("Final Humidity: %.1f%%\n", package["final_humidity"].as<float>());
    Serial.printf("Average Flow Rate: %.3f ml/s\n", package["avg_flow_rate"].as<float>());
    Serial.printf("Duration: %.1f seconds\n", package["duration"].as<float>() / 1000.0f);
    Serial.println("=========================\n");

    return true;
}

size_t DataStorage::getWateringPackageCount() {
    if (!initialized) return 0;
    
    StaticJsonDocument<JSON_SIZE> doc;
    File file = SPIFFS.open(filename, "r");
    if (!file) return 0;
    
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) return 0;
    
    return doc["watering_packages"].size();
}

void DataStorage::calculateWaterUsageStats() {
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

    // Arrays to store sums and counts for each humidity group
    float waterPerHumidityChange[NUM_HUMIDITY_GROUPS] = {0};
    int groupCounts[NUM_HUMIDITY_GROUPS] = {0};

    JsonArray packages = doc["watering_packages"].as<JsonArray>();
    
    Serial.println("\n=== Water Usage Statistics ===");
    
    // Process each package
    for (JsonObject package : packages) {
        float initialHumidity = package["initial_humidity"].as<float>();
        float finalHumidity = package["final_humidity"].as<float>();
        float waterUsed = package["water_used"].as<float>();
        float humidityChange = finalHumidity - initialHumidity;
        
        // Determine which group this belongs to based on initial humidity
        int groupIndex = static_cast<int>(initialHumidity / GROUP_SIZE);
        if (groupIndex >= 0 && groupIndex < NUM_HUMIDITY_GROUPS && humidityChange > 0) {
            waterPerHumidityChange[groupIndex] += (waterUsed / humidityChange);
            groupCounts[groupIndex]++;
        }
    }

    // Print results for each group
    Serial.println("Initial Humidity Range | Avg Water/% Change | Samples");
    Serial.println("--------------------------------------------------");
    for (int i = 0; i < NUM_HUMIDITY_GROUPS; i++) {
        if (groupCounts[i] > 0) {
            float avgWaterPerChange = waterPerHumidityChange[i] / groupCounts[i];
            Serial.printf("%3d%% - %3d%%        | %8.2f ml/%%    | %3d\n",
                        i * 10, (i + 1) * 10,
                        avgWaterPerChange,
                        groupCounts[i]);
        }
    }
    Serial.println("--------------------------------------------------\n");
}

void DataStorage::calculateFlowRateStats() {
    // Reset all group stats
    for (int i = 0; i < NUM_HUMIDITY_GROUPS; i++) {
        groupStats[i] = {0, 0, 0, false};
    }

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

    // Arrays for storing group statistics
    float totalFlowRates[NUM_HUMIDITY_GROUPS] = {0};
    unsigned long totalDurations[NUM_HUMIDITY_GROUPS] = {0};
    int groupCounts[NUM_HUMIDITY_GROUPS] = {0};

    JsonArray packages = doc["watering_packages"].as<JsonArray>();
    
    Serial.println("\n=== Flow Rate Statistics by Humidity Range ===");
    
    // Process each package
    for (JsonObject package : packages) {
        float initialHumidity = package["initial_humidity"].as<float>();
        float avgFlowRate = package["avg_flow_rate"].as<float>();
        unsigned long duration = package["duration"].as<unsigned long>();
        
        int groupIndex = static_cast<int>(initialHumidity / GROUP_SIZE);
        if (groupIndex >= 0 && groupIndex < NUM_HUMIDITY_GROUPS) {
            totalFlowRates[groupIndex] += avgFlowRate;
            totalDurations[groupIndex] += duration;
            groupCounts[groupIndex]++;
        }
    }

    // Print results for each group
    Serial.println("Init Humidity | Flow Rate (ml/s) | Avg Time (s) | Samples");
    Serial.println("--------------------------------------------------");
    for (int i = 0; i < NUM_HUMIDITY_GROUPS; i++) {
        if (groupCounts[i] > 0) {
            float avgFlowRate = totalFlowRates[i] / groupCounts[i];
            float avgSeconds = (totalDurations[i] / 1000.0f) / groupCounts[i];
            
            // Store stats in member variable
            groupStats[i].avgFlowRate = avgFlowRate;
            groupStats[i].avgTime = avgSeconds;
            groupStats[i].samples = groupCounts[i];
            groupStats[i].hasData = true;

            Serial.printf("%2d%% - %3d%% | %10.3f     | %7.1f    | %3d\n",
                        i * 10, (i + 1) * 10,
                        avgFlowRate,
                        avgSeconds,
                        groupCounts[i]);
        }
    }
    Serial.println("--------------------------------------------------");
}

HumidityGroupStats DataStorage::getStatsForHumidityRange(float humidity) const {
    int groupIndex = static_cast<int>(humidity / GROUP_SIZE);
    if (groupIndex >= 0 && groupIndex < NUM_HUMIDITY_GROUPS) {
        return groupStats[groupIndex];
    }
    return {0, 0, 0, false};
}