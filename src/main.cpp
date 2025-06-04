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
#include "weather_service.h"
#include "esp_wifi.h"

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
WateringSystem pump(80.0f, 80.0f, 27.0f, 30.0f);
SHTC3 shtc3(&Serial2, SERIAL1_TX, SERIAL1_RX, MODBUS_DE_PINOUT, MODBUS_RE_PINOUT, 1, SERIAL_MODBUS_BAUD);

uint32_t previousStateChange;

uint32_t previousDataSend;

bool wateringFlag = false;

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
} gw_to_sensor_node_msg;

gw_to_sensor_node_msg incomingData;

void onDataRecv(const uint8_t *mac, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  wateringFlag = incomingData.watering;
  Serial.printf("Received -> watering: %d\n",
                incomingData.watering);
}

#define WATER_PUMP_PIN D9
#define LIGHT_SENSOR_PIN A3
#define LED_PIN D8

void TaskReadAndSendTelemetryData(void *pvParameters) {
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

    // Water pump control
    //waterPump.pump(brightness / 4095.0 * 255);

    dataToSend.temperature = temperature;
    dataToSend.humidity = humidity;
    
    vTaskDelay(pdMS_TO_TICKS(SEND_TELEMETRY_INTERVAL_MS));
  }
}

void TaskWatering(void *pvParameters) {
  while(1) {
    if (wateringFlag) {
      Serial.println("------------------");
      Serial.println("Watering triggered by RPC call");
      bool humidityStatus = false;
      bool temperatureStatus = false;
      pump.watering(shtc3.getHumidity(humidityStatus),
                    80.0f,
                    shtc3.getTemperature(temperatureStatus),
                    30.0f);

      
      // float temperature, humidity;
      // tempHumidSensor.get_value(temperature, humidity);
      // pump.watering(humidity,
      //               80.0f,
      //               temperature,
      //               30.0f);
      
      // waterPump.pump(pump.getFlowRate() / 10000.0 * 255);
      if (!pump.isWatering()) {
        wateringFlag = false;
        // waterPump.pump(0);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Simulate watering every 1 seconds
  }
}

void TaskSHTC3Read(void *pvParameters) {
  while(1) {
    bool humidityStatus = false;
    bool temperatureStatus = false;
    float soilHumidity = shtc3.getHumidity(humidityStatus);
    float soilTemperature = shtc3.getTemperature(temperatureStatus);

    if (humidityStatus == MODBUS_OK) {
      Serial.println("------------------");
      Serial.print("SHTC3 Soil Humidity: ");
      Serial.print(soilHumidity);
      Serial.println("%");

      dataToSend.soil_moisture = soilHumidity;
      esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&dataToSend, sizeof(dataToSend));
      if (result != ESP_OK) {
        Serial.println("Error sending the data");
      }
    } else {
      Serial.println("------------------");
      Serial.println("Failed to read Soil Humidity from SHTC3 sensor!");
    }

    if (temperatureStatus == MODBUS_OK) {
      Serial.println("------------------");
      Serial.print("SHTC3 Soil Temperature: ");
      Serial.print(soilTemperature);
      Serial.println(" °C");
    } else {
      Serial.println("------------------");
      Serial.println("Failed to read Soil Temperature from SHTC3 sensor!");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
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
  
  // Increase stack sizes and adjust priorities
  xTaskCreate(TaskSHTC3Read, "SHTC3 Read Task", 4096, NULL, 2, NULL);
  xTaskCreate(TaskWatering, "Watering task", 4096, NULL, 3, NULL);
  xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
}

void loop() {
}
