#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "temp_humid.h"
#include "light.h"
#include "water_pump.h"
#include <ArduinoOTA.h>
#include <esp_smartconfig.h>
#include <esp_now.h>

constexpr char WIFI_SSID[] = "MSI";
constexpr char WIFI_PASSWORD[] = "25122003";

constexpr char TOKEN[] = "gd63iniu7du1xm4zetoh";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;


constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
constexpr uint16_t SEND_TELEMETRY_INTERVAL_MS = 1000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;

#define WATER_PUMP_PIN 14
#define LIGHT_SENSOR_PIN 36

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

TempHumidSensor tempHumidSensor;
LightSensor lightSensor;
WaterPump waterPump;

// ESP-NOW receiving
typedef struct struct_message {
  int id;
  float temperature;
  float humidity;
} struct_message;

struct_message incomingData;

void onDataRecv(const uint8_t *mac, const uint8_t *incoming, int len) {
  memcpy(&incomingData, incoming, sizeof(incomingData));
  Serial.printf("Received -> ID: %d, Temp: %.2f, Hum: %.2f\n",
                incomingData.id, incomingData.temperature, incomingData.humidity);
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
    vTaskDelay(pdMS_TO_TICKS(TB_CONNECT_CHECKING_INTERVAL_MS));
  }
}

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
    uint32_t brightness = lightSensor.getBrightness();
    Serial.print("Brightness: ");
    Serial.println(brightness);
    // tb.sendTelemetryData("brightness", brightness);

    // Water pump control
    waterPump.pump(brightness / 4095.0 * 255);

    vTaskDelay(pdMS_TO_TICKS(SEND_TELEMETRY_INTERVAL_MS));
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();
  tempHumidSensor.begin();
  lightSensor.begin(LIGHT_SENSOR_PIN);
  waterPump.begin(WATER_PUMP_PIN);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);

  xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCheckTBConnection, "Check Thingsboard connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
}

void loop() {
  
}
