#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "temp_humid.h"
#include "light.h"
#include "water_pump.h"
#include <ArduinoOTA.h>
#include "scheduler.h"
#include "watering.h"
#include <Attribute_Request.h>
#include <OTA_Firmware_Update.h>
#include <Server_Side_RPC.h>
#include <Shared_Attribute_Update.h>
#include "shtc3.h"
#include <esp_smartconfig.h>
#include <esp_now.h>
#include <Espressif_Updater.h>
#include "esp_wifi.h"
#include "data_storage.h"
#include <time.h>

#define SERIAL1_TX 16
#define SERIAL1_RX 17
#define MODBUS_DE_PINOUT 27
#define MODBUS_RE_PINOUT 14


constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
constexpr uint16_t OTA_UPDATE_INTERVAL_MS = 100U;
constexpr uint16_t SEND_TELEMETRY_INTERVAL_MS = 5000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr uint32_t SERIAL_MODBUS_BAUD = 4800U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;


volatile bool attributesChanged = false;
volatile bool shared_update_subscribed = false;
volatile bool rpc_server_subscribed = false;
volatile bool request_send = false;
volatile int wateringMode = 0;
volatile bool wateringState = false;

constexpr uint16_t WATERING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t WATERING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t wateringInterval = 1000U;

DHT20 dht20;
WateringSystem pump(80.0f);
SHTC3 shtc3(&Serial2, SERIAL1_TX, SERIAL1_RX, MODBUS_DE_PINOUT, MODBUS_RE_PINOUT, 1, SERIAL_MODBUS_BAUD);
DataStorage storage;

uint32_t previousStateChange;

uint32_t previousDataSend;

bool wateringFlag = false;
bool clearDataFlag = false;

TempHumidSensor tempHumidSensor;
LightSensor  lightSensor;
WaterPump waterPump;
uint8_t receiverMAC[] = {0xE8, 0x68, 0xe7, 0x19, 0x27, 0x2c};

typedef struct sensor_node_to_gw_msg {
  float temperature;
  float humidity;
  float soil_moisture;
} sensor_node_to_gw_msg;

sensor_node_to_gw_msg dataToSend;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}
typedef struct gw_to_sensor_node_msg {
  bool watering;
  bool clearData;
} gw_to_sensor_node_msg;

gw_to_sensor_node_msg incomingData;

void onDataRecv(const uint8_t *mac, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  wateringFlag = incomingData.watering;
  clearDataFlag = incomingData.clearData;
  Serial.printf("Received -> watering: %d, clearData: %d\n",
                incomingData.watering, incomingData.clearData);
}

#define WATER_PUMP_PIN D9
#define LIGHT_SENSOR_PIN A3
#define LED_PIN D8

void TaskDHT20LightSensorRead(void *pvParameters) {
  while(1) {
    
    // Humidity and temperature
    float temperature = 25;//dht20.getTemperature();
    float humidity = 50;//dht20.getHumidity();

    tempHumidSensor.get_value(temperature, humidity);

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      // tb.sendTelemetryData("temperature", temperature);
      // tb.sendTelemetryData("humidity", humidity);
    }

    // Brightness
    lightSensor.controlLED();

    dataToSend.temperature = temperature;
    dataToSend.humidity = humidity;
    
    vTaskDelay(pdMS_TO_TICKS(SEND_TELEMETRY_INTERVAL_MS));
  }
}

float soilHumidity = 0;
float soilTemperature = 0;
HumidityGroupStats humidityStats[10];  // Array to store stats for all groups
bool statsUpdated = false;

void TaskSHTC3Read(void *pvParameters) {
    static float initialHumidity = 0;
    static bool isWateringStart = false;

    while(1) {
        bool humidityStatus = false;
        bool temperatureStatus = false;
        soilHumidity = shtc3.getHumidity(humidityStatus);
        soilTemperature = shtc3.getTemperature(temperatureStatus);

        if (humidityStatus == MODBUS_OK && temperatureStatus == MODBUS_OK) {
            // // Store data only at the start and end of watering cycle
            // if (wateringFlag && !isWateringStart) {
            //     // Beginning of watering cycle
            //     initialHumidity = soilHumidity;
            //     isWateringStart = true;
            //     storage.storeSensorData(soilHumidity, soilTemperature);
            //     Serial.println("------------------");
            //     Serial.println("Initial watering data stored");
            // } else if (!wateringFlag && isWateringStart) {
            //     // End of watering cycle
            //     storage.storeSensorData(soilHumidity, soilTemperature);
            //     Serial.println("------------------");
            //     Serial.println("Final watering data stored");
            //     isWateringStart = false;
            // }

            // Always send telemetry data
            Serial.println("------------------");
            Serial.print("SHTC3 Soil Humidity: ");
            Serial.print(soilHumidity);
            Serial.println("%");
            dataToSend.soil_moisture = soilHumidity;
            esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
            if (result != ESP_OK) {
              Serial.println("Error sending the data");
            }

            Serial.print("SHTC3 Soil Temperature: ");
            Serial.println(soilTemperature);
            //tb.sendTelemetryData("soilTemperature", soilTemperature);
        } else {
            Serial.println("------------------");
            Serial.println("Failed to read Soil Humidity or Temperature from SHTC3 sensor!");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

constexpr uint32_t STATS_UPDATE_INTERVAL_MS = 10000; // 10 seconds
constexpr uint8_t PACKAGES_PER_PAGE = 3;  // Number of packages to show per interval

void TaskPeriodicStats(void *pvParameters) {
    while(1) {
        Serial.println("\n=== Storage Statistics ===");
        
        // Show storage info
        storage.printStorageStats();
        
        // Show latest watering package and calculate statistics
        size_t packageCount = storage.getWateringPackageCount();
        if (packageCount > 0) {
            // Calculate and show flow rate statistics by humidity range
            storage.calculateFlowRateStats();
            
            // Get updated stats
            const HumidityGroupStats* newStats = storage.getAllStats();
            // Copy to global array
            for (int i = 0; i < DataStorage::NUM_HUMIDITY_GROUPS; i++) {
                humidityStats[i] = newStats[i];
            }
            statsUpdated = true;
            
            // Print latest package
            storage.printWateringPackage(0);
        }
        
        Serial.println("\nSystem Uptime:");
        unsigned long uptime = millis() / 1000;
        Serial.printf("Running for: %02lu:%02lu:%02lu\n", 
            uptime / 3600, (uptime % 3600) / 60, uptime % 60);
        
        Serial.println("========================\n");
        vTaskDelay(pdMS_TO_TICKS(STATS_UPDATE_INTERVAL_MS));
    }
}

void TaskClearData(void *pvParameters) {
    while(1) {
        if (clearDataFlag) {
            Serial.println("------------------");
            Serial.println("Clearing storage data...");
            
            if (storage.clearStorage()) {
                Serial.println("Storage cleared successfully");
            } else {
                Serial.println("Failed to clear storage!");
            }
            
            clearDataFlag = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskWatering(void *pvParameters) {
    while(1) {
        if (wateringFlag) {
            static float initialHumidity = 0;
            static bool cycleStarted = false;
            
            // Store initial data when starting a new watering cycle
            if (!cycleStarted) {
                initialHumidity = soilHumidity;
                
                // Get stats for current humidity level
                // int groupIndex = static_cast<int>(soilHumidity / 10.0f);
                // if (statsUpdated && humidityStats[groupIndex].hasData) {
                //     Serial.printf("Historical data for %d%% - %d%% humidity:\n", 
                //                 groupIndex * 10, (groupIndex + 1) * 10);
                //     Serial.printf("Avg Flow Rate: %.1f ml/h\n", humidityStats[groupIndex].avgFlowRate);
                //     Serial.printf("Avg Time: %.1f s\n", humidityStats[groupIndex].avgTime);
                //     Serial.printf("Sample Count: %d\n", humidityStats[groupIndex].samples);
                // }
                
                cycleStarted = true;
                Serial.println("------------------");
                Serial.println("Starting watering cycle");
                Serial.printf("Initial soil humidity: %.1f%%\n", initialHumidity);
            }
            
            // Normal watering process with historical data
            pump.watering(soilHumidity, humidityStats[static_cast<int>(soilHumidity / 10.0f)]);
            Serial.printf("Flow rate: %.3f", pump.getFlowRate());
                         
            // Store complete package when watering cycle ends
            if (!pump.isWatering() && wateringFlag && cycleStarted) {
                float finalHumidity = soilHumidity;
                float avgFlowRate = pump.getAverageFlowRate(); // Keep in ml/h
                unsigned long duration = pump.getWateringDuration();
                
                // Store complete watering package
                storage.storeWateringPackage(
                    initialHumidity,
                    finalHumidity,
                    avgFlowRate,
                    duration
                );
                
                Serial.println("------------------");
                Serial.println("Watering cycle completed");
                Serial.printf("Initial humidity: %.1f%%\n", initialHumidity);
                Serial.printf("Final humidity: %.1f%%\n", finalHumidity);
                Serial.printf("Average flow rate: %.1f ml/h\n", avgFlowRate);
                Serial.printf("Duration: %.1f seconds\n", duration / 1000.0f);
                
                wateringFlag = false;
                cycleStarted = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);

  tempHumidSensor.begin(); 
  lightSensor.begin(LIGHT_SENSOR_PIN, LED_PIN);
  
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_ps(WIFI_PS_NONE);
  
  // Get and print MAC address
  String mac = WiFi.macAddress();
  Serial.print("ESP32 MAC Address: ");
  Serial.println(mac);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 6;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  
  if (!storage.begin()) {
        Serial.println("Failed to initialize storage!");
    }
    
  // Task creation with optimized stack sizes
    
    // Medium priority sensor and control tasks
    xTaskCreate(TaskSHTC3Read, "SHTC3 Read Task", 4096, NULL, 2, NULL);
    xTaskCreate(TaskWatering, "Watering task", 16384, NULL, 3, NULL);
    xTaskCreate(TaskDHT20LightSensorRead, "DHT20 and Light Sensor Read Task", 16384, NULL, 3, NULL);
    xTaskCreate(TaskClearData, "Clear Data", 16384, NULL, 2, NULL);        // Add this line
    
    // Low priority monitoring task
    xTaskCreate(TaskPeriodicStats, "Periodic Stats", 16384, NULL, 1, NULL);
}

void loop() {}