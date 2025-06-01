#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include "scheduler.h"
#include "watering.h"
#include <Attribute_Request.h>
#include <OTA_Firmware_Update.h>
#include <Server_Side_RPC.h>
#include <Shared_Attribute_Update.h>
#include "shtc3.h"
#include "weather_service.h"
#include "data_storage.h"

#define SERIAL1_TX 16
#define SERIAL1_RX 17
#define MODBUS_DE_PINOUT 27
#define MODBUS_RE_PINOUT 14

constexpr char WIFI_SSID[] = "Oreki";
constexpr char WIFI_PASSWORD[] = "hardware";

constexpr char TOKEN[] = "I6CQCDQxVfWAU2uezJeZ";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;


constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
constexpr uint16_t SEND_TELEMETRY_INTERVAL_MS = 1000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr uint32_t SERIAL_MODBUS_BAUD = 4800U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;

// Scheduler
constexpr char WATERING_INTERVAL_ATTR[] = "wateringInterval";
constexpr char WATERING_MODE_ATTR[] = "wateringMode";
constexpr char WATERING_STATE_ATTR[] = "wateringState";
constexpr uint8_t MAX_ATTRIBUTES = 2;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 10000U * 1000U;
void requestTimedOut() {
  Serial.printf("Attribute request timed out did not receive a response in (%llu) microseconds. Ensure client is connected to the MQTT broker and that the keys actually exist on the target device\n", REQUEST_TIMEOUT_MICROSECONDS);
}

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
WateringSystem pump(70.0f, 80.0f, 27.0f, 30.0f);
SHTC3 shtc3(&Serial2, SERIAL1_TX, SERIAL1_RX, MODBUS_DE_PINOUT, MODBUS_RE_PINOUT, 1, SERIAL_MODBUS_BAUD);
DataStorage storage;

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  WATERING_STATE_ATTR,
  WATERING_INTERVAL_ATTR
};
bool wateringFlag = false;
void watering(const JsonVariantConst& variant, JsonDocument& document) {
  Serial.println("watering is called");
  wateringFlag = true;
  const size_t jsonSize = Helper::Measure_Json(variant);
  char buffer[jsonSize];
  serializeJson(variant, buffer, jsonSize);
  Serial.println(buffer);
}

const std::array<RPC_Callback, 1U> RPC_CALLBACK = {
    RPC_Callback{"watering", watering}
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
OTA_Firmware_Update<> ota;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Server_Side_RPC<> server_side_rpc;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
const std::array<IAPI_Implementation*, 4U> apis = {
    &shared_update,
    &attr_request,
    &server_side_rpc,
    &ota
};

// Initialize ThingsBoard instance with the maximum needed buffer size
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE, MAX_MESSAGE_SIZE, Default_Max_Stack_Size, apis);
// ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

// void processSharedAttributes(const Shared_Attribute_Data &data) {
//   for (auto it = data.begin(); it != data.end(); ++it) {
//     if (strcmp(it->key().c_str(), WATERING_INTERVAL_ATTR) == 0) {
//       const uint16_t new_interval = it->value().as<uint16_t>();
//       if (new_interval >= WATERING_INTERVAL_MS_MIN && new_interval <= WATERING_INTERVAL_MS_MAX) {
//         wateringInterval = new_interval;
//         Serial.print("Watering interval is set to: ");
//         Serial.println(new_interval);
//       }
//     } else if (strcmp(it->key().c_str(), WATERING_STATE_ATTR) == 0) {
//       wateringState = it->value().as<bool>();
//       // digitalWrite(LED_PIN, wateringState);
//       Serial.print("Watering state is set to: ");
//       Serial.println(wateringState);
//     }
//   }
//   attributesChanged = true;
// }

// const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
// const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void processSharedAttribute(const JsonObjectConst &data) {
  Serial.println("Received Shared Attributes...");

  // Iterate through each key-value pair
  for (JsonPairConst kv : data) {
    const char* key = kv.key().c_str();
    if (strcmp(key, WATERING_STATE_ATTR) == 0) {
      wateringState = kv.value().as<bool>();
    }
  }
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  // Attempting to establish a connection to the given WiFi network
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    // Delay 500ms until a connection has been successfully established
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void CheckTBConnection() {
  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    Serial.println("Connect successfully");
  }
}

/* Free RTOS Tasks */
void TaskCheckWiFiConnection(void *pvParameters) {
  while(1) {
    // Check to ensure we aren't connected yet
    const wl_status_t status = WiFi.status();
    if (status != WL_CONNECTED) {
      // If we aren't establish a new connection to the given WiFi network
      InitWiFi();
      // Reconnect to ThingsBoard right after reconnect to WiFi
      CheckTBConnection(); 
    }
    vTaskDelay(pdMS_TO_TICKS(WIFI_CONNECT_CHECKING_INTERVAL_MS));
  }
}

void TaskCheckTBConnection(void *pvParameters) {
  while(1) {
    CheckTBConnection();

    if (!shared_update_subscribed){
      Serial.println("Subscribing for shared attribute updates...");
      const Shared_Attribute_Callback<MAX_ATTRIBUTES> callback(&processSharedAttribute, SHARED_ATTRIBUTES_LIST);
      if (!shared_update.Shared_Attributes_Subscribe(callback)) {
        Serial.println("Failed to subscribe for shared attribute updates");
        // continue;
      }
      Serial.println("Subscribe done");
      shared_update_subscribed = true;
    }

    if (!request_send) {
      Serial.println("Requesting shared attributes...");
      const Attribute_Request_Callback<MAX_ATTRIBUTES> sharedCallback(&processSharedAttribute, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES_LIST);
      request_send = attr_request.Shared_Attributes_Request(sharedCallback);
      if (!request_send) {
        Serial.println("Failed to request shared attributes");
      }
    }

    if (!rpc_server_subscribed) {
      Serial.println("Subscribing server-side RPC");
      rpc_server_subscribed = server_side_rpc.RPC_Subscribe(RPC_CALLBACK.cbegin(), RPC_CALLBACK.cend());
      if (!rpc_server_subscribed) {
        Serial.println("Failed to subscribe rpc");
      }
    }

    // if (!tb.Shared_Attributes_Subscribe(attributes_callback)) {
    //     Serial.println("Failed to subscribe for shared attribute updates");
    //     return;
    // }
    vTaskDelay(pdMS_TO_TICKS(TB_CONNECT_CHECKING_INTERVAL_MS));
  }
}

void TaskReadAndSendTelemetryData(void *pvParameters) {
  while(1) {
    // dht20.read();
    
    float temperature = 25;//dht20.getTemperature();
    float humidity = 50;//dht20.getHumidity();

    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT20 sensor!");
    } else {
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");

      tb.sendTelemetryData("temperature", temperature);
      tb.sendTelemetryData("humidity", humidity);
    }
    vTaskDelay(pdMS_TO_TICKS(SEND_TELEMETRY_INTERVAL_MS));
  }
}

void TaskTBloop(void *pvParameters) {
  while(1) {
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(TB_LOOP_INTERVAL_MS));
  }
}

void TaskWatering(void *pvParameters) {
    while(1) {
        if (wateringFlag) {
            static float initialHumidity = 0;
            static uint32_t startTime = 0;
            static bool cycleStarted = false;
            bool humidityStatus = false;
            bool temperatureStatus = false;
            
            // Store initial data when starting a new watering cycle
            if (!cycleStarted) {
                initialHumidity = shtc3.getHumidity(humidityStatus);
                if (humidityStatus == MODBUS_OK) {
                    startTime = millis();
                    cycleStarted = true;
                    Serial.println("------------------");
                    Serial.println("Starting watering cycle");
                    Serial.printf("Initial soil humidity: %.1f%%\n", initialHumidity);
                }
            }
            
            // Normal watering process
            pump.watering(shtc3.getHumidity(humidityStatus),
                         80.0f,
                         shtc3.getTemperature(temperatureStatus),
                         30.0f);
                         
            // Store complete package when watering cycle ends
            if (!pump.isWatering() && wateringFlag && cycleStarted) {
                float finalHumidity = shtc3.getHumidity(humidityStatus);
                if (humidityStatus == MODBUS_OK) {
                    // Store complete watering package
                    storage.storeWateringPackage(
                        startTime,
                        initialHumidity,
                        finalHumidity,
                        pump.getWaterUsed()
                    );
                    
                    Serial.println("------------------");
                    Serial.println("Watering cycle completed");
                    Serial.printf("Initial humidity: %.1f%%\n", initialHumidity);
                    Serial.printf("Final humidity: %.1f%%\n", finalHumidity);
                    Serial.printf("Total water used: %.2f ml\n", pump.getWaterUsed());
                }
                wateringFlag = false;
                cycleStarted = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void TaskSHTC3Read(void *pvParameters) {
    static float initialHumidity = 0;
    static bool isWateringStart = false;

    while(1) {
        bool humidityStatus = false;
        bool temperatureStatus = false;
        float soilHumidity = shtc3.getHumidity(humidityStatus);
        float soilTemperature = shtc3.getTemperature(temperatureStatus);

        if (humidityStatus == MODBUS_OK && temperatureStatus == MODBUS_OK) {
            // Store data only at the start and end of watering cycle
            if (wateringFlag && !isWateringStart) {
                // Beginning of watering cycle
                initialHumidity = soilHumidity;
                isWateringStart = true;
                storage.storeSensorData(soilHumidity, soilTemperature);
                Serial.println("------------------");
                Serial.println("Initial watering data stored");
            } else if (!wateringFlag && isWateringStart) {
                // End of watering cycle
                storage.storeSensorData(soilHumidity, soilTemperature);
                Serial.println("------------------");
                Serial.println("Final watering data stored");
                isWateringStart = false;
            }

            // Always send telemetry data
            Serial.println("------------------");
            Serial.print("SHTC3 Soil Humidity: ");
            Serial.print(soilHumidity);
            Serial.println("%");
            tb.sendTelemetryData("soilHumidity", soilHumidity);

            Serial.print("SHTC3 Soil Temperature: ");
            Serial.print(soilTemperature);
            Serial.println(" °C");
            tb.sendTelemetryData("soilTemperature", soilTemperature);
        } else {
            Serial.println("------------------");
            Serial.println("Failed to read Soil Humidity or Temperature from SHTC3 sensor!");
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

constexpr char WEATHER_API_KEY[] = "2feda8f965ede8af1b286418cfe18a5e";  // Replace with your actual OpenWeatherMap API key
constexpr float LATITUDE = 10.87999801f;
constexpr float LONGITUDE = 106.80634192f;
constexpr uint32_t WEATHER_UPDATE_INTERVAL_MS = 300000; // 5 minutes

WeatherService weatherService(WEATHER_API_KEY, LATITUDE, LONGITUDE);

void TaskWeatherUpdate(void *pvParameters) {
    while(1) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("------------------");
            Serial.println("Updating weather data...");
            if (weatherService.getCurrentWeather()) {
                Serial.println("------------------");
                Serial.println("Weather Data:");
                Serial.printf("Temperature: %.1f°C\n", weatherService.getTemperature());
                Serial.printf("Feels Like: %.1f°C\n", weatherService.getFeelsLike());
                Serial.printf("Pressure: %.0f hPa\n", weatherService.getPressure());
                Serial.printf("Humidity: %.0f%%\n", weatherService.getHumidity());
                Serial.printf("Wind Speed: %.1f m/s\n", weatherService.getWindSpeed());
                
                float windGust = weatherService.getWindGust();
                if (windGust > 0) {
                    Serial.printf("Wind Gust: %.1f m/s\n", windGust);
                }
                
                float rain = weatherService.getRain();
                if (rain > 0) {
                    Serial.printf("Rain (1h): %.2f mm\n", rain);
                }
                
                Serial.printf("Clouds: %.0f%%\n", weatherService.getClouds());
                
                // Send telemetry to ThingsBoard
                weatherService.sendWeatherTelemetry(tb);
                Serial.println("Weather data sent to ThingsBoard");
            } else {
                Serial.println("Failed to get weather data!");
            }
        } else {
            Serial.println("WiFi not connected, skipping weather update");
        }
        vTaskDelay(pdMS_TO_TICKS(WEATHER_UPDATE_INTERVAL_MS));
    }
}

void showStorageStats() {
    Serial.println("\n=== Storage Statistics ===");
    
    bool humidityStatus = false;
    bool temperatureStatus = false;
    float currentHumidity = shtc3.getHumidity(humidityStatus);
    float currentTemperature = shtc3.getTemperature(temperatureStatus);
    
    // Serial.println("\nCurrent Sensor Values:");
    // if (humidityStatus == MODBUS_OK && temperatureStatus == MODBUS_OK) {
    //     Serial.printf("Soil Humidity: %.1f%%\n", currentHumidity);
    //     Serial.printf("Soil Temperature: %.1f°C\n", currentTemperature);
    // } else {
    //     Serial.println("Failed to read current sensor values");
    // }
    
    // Serial.println("\nWatering Status:");
    // Serial.printf("Watering Active: %s\n", wateringFlag ? "Yes" : "No");
    // if (wateringFlag) {
    //     Serial.printf("Water Used: %.2f ml\n", pump.getWaterUsed());
    // }
    
    Serial.println("\nStorage Info:");
    storage.printStorageStats();
    
    Serial.println("\nSystem Uptime:");
    unsigned long uptime = millis() / 1000; // Convert to seconds
    Serial.printf("Running for: %02lu:%02lu:%02lu\n", 
        uptime / 3600, (uptime % 3600) / 60, uptime % 60);
    
    Serial.println("========================\n");
}

constexpr uint32_t STATS_UPDATE_INTERVAL_MS = 10000; // 10 seconds

void TaskPeriodicStats(void *pvParameters) {
    while(1) {
        showStorageStats();
        vTaskDelay(pdMS_TO_TICKS(STATS_UPDATE_INTERVAL_MS));
    }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  // Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for Serial2
  // shct3 = SHCT3(&Serial2, 1, SERIAL_MODBUS_BAUD);
  delay(1000);
  InitWiFi();

  Wire.begin();
  dht20.begin();
  
  if (!storage.begin()) {
        Serial.println("Failed to initialize storage!");
    }
    
  // Task creation with optimized stack sizes
    // High priority network tasks
    xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 3072, NULL, 4, NULL);      // Reduced - simple WiFi checks
    xTaskCreate(TaskCheckTBConnection, "Check TB connection", 4096, NULL, 4, NULL);          // Reduced - basic TB connection
    xTaskCreate(TaskTBloop, "ThingsBoard loop", 4096, NULL, 3, NULL);                       // Kept same - TB communication
    
    // Medium priority sensor and control tasks
    xTaskCreate(TaskSHTC3Read, "SHTC3 Read Task", 4096, NULL, 2, NULL);                     // Reduced - simple sensor reading
    xTaskCreate(TaskWatering, "Watering task", 8192, NULL, 3, NULL);                        // Reduced - watering control
    xTaskCreate(TaskWeatherUpdate, "Weather Update Task", 8192, NULL, 2, NULL);             // Kept same - JSON parsing
    
    // Low priority monitoring task
    xTaskCreate(TaskPeriodicStats, "Periodic Stats", 4096, NULL, 1, NULL);                  // Reduced - simple stats display
}

void loop() {
}
