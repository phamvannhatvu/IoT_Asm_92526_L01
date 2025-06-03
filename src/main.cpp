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
#include "weather_service.h"

#define SMARTCONFIG_TIMEOUT 1

constexpr char WIFI_SSID[] = "Oreki";
constexpr char WIFI_PASSWORD[] = "hardware";

constexpr char TOKEN[] = "gd63iniu7du1xm4zetoh";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;


constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
constexpr uint16_t OTA_UPDATE_INTERVAL_MS = 100U;
constexpr uint16_t SEND_TELEMETRY_INTERVAL_MS = 5000U;
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

uint32_t previousStateChange;

constexpr int16_t telemetrySendInterval = 10000U;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  WATERING_STATE_ATTR,
  WATERING_INTERVAL_ATTR
};


uint8_t receiverMAC[] = {0xB4, 0xE6, 0x2D, 0xD5, 0x5A, 0x55};

typedef struct sensor_node_to_gw_msg {
  float temperature;
  float humidity;
  float soil_moisture;
} sensor_node_to_gw_msg;

sensor_node_to_gw_msg incomingData;

void onDataRecv(const uint8_t *mac, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  Serial.printf("Received -> temperature: %.2f, humidity: %.2f, soil_moisture: %.2f\n",
                incomingData.temperature, incomingData.humidity, incomingData.soil_moisture);
}

typedef struct gw_to_sensor_node_msg {
  bool watering;
} gw_to_sensor_node_msg;

gw_to_sensor_node_msg dataToSend;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void watering(const JsonVariantConst& variant, JsonDocument& document) {
  Serial.println("watering is called");
  dataToSend.watering = true;
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
  if (result != ESP_OK) {
    Serial.println("Error sending the data");
  }

  const size_t jsonSize = Helper::Measure_Json(variant);
  char buffer[jsonSize];
  serializeJson(variant, buffer, jsonSize);
  Serial.println(buffer);
}

const std::array<RPC_Callback, 1U> RPC_CALLBACK = {
    RPC_Callback{"watering", watering}
};

// Initalize the Updater client instance used to flash binary to flash memory
Espressif_Updater<> updater;
// Statuses for updating
bool currentFWSent = false;
bool updateRequestSent = false;
// Initialize used apis
OTA_Firmware_Update<> ota;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;

// Firmware title and version used to compare with remote version, to check if an update is needed.
// Title needs to be the same and version needs to be different --> downgrading is possible
constexpr char CURRENT_FIRMWARE_TITLE[] = "SmartAgriculture";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0";
// Maximum amount of retries we attempt to download each firmware chunck over MQTT
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
// Size of each firmware chunck downloaded over MQTT,
// increased packet size, might increase download speed
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

// Initialize ThingsBoard instance with the maximum needed buffer size
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 512U;
WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
Server_Side_RPC<> server_side_rpc;
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
  WiFi.mode(WIFI_STA);

  // Start SmartConfig
  WiFi.beginSmartConfig();
  Serial.println("Waiting for SmartConfig...");

  unsigned long startTime = millis();
  bool smartConfigReceived = false;

  // Wait for SmartConfig or timeout
  while (millis() - startTime < SMARTCONFIG_TIMEOUT * 1000) {
    if (WiFi.smartConfigDone()) {
      smartConfigReceived = true;
      Serial.println("SmartConfig received.");
      break;
    }
    delay(500);
    Serial.print(".");
  }

  if (smartConfigReceived) {
    // Wait for Wi-Fi to connect using SmartConfig credentials
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print("*");
    }
    Serial.println("\nConnected to Wi-Fi via SmartConfig: " + WiFi.SSID());
  } else {
    Serial.println("\nSmartConfig timeout. Attempting to reconnect to saved Wi-Fi...");

    WiFi.stopSmartConfig();  // Stop SmartConfig if active
    WiFi.begin();            // Attempt connection using saved credentials

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED && retry++ < 20) {
      delay(500);
      Serial.print("#");
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nReconnected to saved Wi-Fi: " + WiFi.SSID());
    } else {
      Serial.println("\nFailed to connect to any Wi-Fi network.");
    }
  }
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

// OTA Callbacks
void update_starting_callback() {
  Serial.println("Start OTA updating");
}

void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    esp_restart();
    return;
  }
  Serial.println("Downloading firmware failed");
  updateRequestSent = false;
}

void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}

/* Free RTOS Tasks */
void TaskOTAUpdate(void *pvParameters) {
  while(1) {
    if (tb.connected())
    {
      if (!currentFWSent) {
        currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
      }

      if (!updateRequestSent) {
        Serial.print(CURRENT_FIRMWARE_TITLE);
        Serial.println(CURRENT_FIRMWARE_VERSION);
        Serial.println("Firwmare Update ...");
        const OTA_Update_Callback callback(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION, &updater, &finished_callback, &progress_callback, &update_starting_callback, FIRMWARE_FAILURE_RETRIES, FIRMWARE_PACKET_SIZE);
        updateRequestSent = ota.Start_Firmware_Update(callback);
        if(updateRequestSent) {
          delay(500);
          Serial.println("Firwmare Update Subscription...");
          updateRequestSent = ota.Subscribe_Firmware_Update(callback);
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(OTA_UPDATE_INTERVAL_MS));
  }
}

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
    
    // Humidity and temperature
    float temperature = incomingData.temperature;
    float humidity = incomingData.humidity;
    float soil_moisture = incomingData.soil_moisture;

    tb.sendTelemetryData("temperature", temperature);
    tb.sendTelemetryData("humidity", humidity);
    tb.sendTelemetryData("soil_moisture", soil_moisture);

    vTaskDelay(pdMS_TO_TICKS(SEND_TELEMETRY_INTERVAL_MS));
  }
}

void TaskTBloop(void *pvParameters) {
  while(1) {
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(TB_LOOP_INTERVAL_MS));
  }
}

constexpr char WEATHER_API_KEY[] = "2feda8f965ede8af1b286418cfe18a5e";  // Replace with your actual OpenWeatherMap API key
constexpr float LATITUDE = 10.87999801f;
constexpr float LONGITUDE = 106.80634192f;
constexpr uint32_t WEATHER_UPDATE_INTERVAL_MS = 300000; // 5 seconds

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

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  // Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX, TX pins for Serial2
  // shct3 = SHCT3(&Serial2, 1, SERIAL_MODBUS_BAUD);
  delay(1000);
  InitWiFi();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  dataToSend.watering = false;

  // for (uint16_t i = 0; i < 256; ++i) {
  //   delay(100);
  //   waterPump.pump(i);
  // }
  // waterPump.pump(10);
  
  // Print current Wi-Fi channel
  uint8_t channel;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&channel, &second);
  Serial.printf("Wi-Fi channel: %d\n", channel);
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_wifi_set_ps(WIFI_PS_NONE);

  // Increase stack sizes and adjust priorities
  xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 4096, NULL, 4, NULL);
  xTaskCreate(TaskCheckTBConnection, "Check Thingsboard connection", 8192, NULL, 4, NULL);
  xTaskCreate(TaskTBloop, "ThingsBoard loop", 4096, NULL, 3, NULL);
  xTaskCreate(TaskWeatherUpdate, "Weather Update Task", 8192, NULL, 2, NULL);
  xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
}

void loop() {
}
