#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include "shtc3.h"

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

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);

DHT20 dht20;
SHTC3 shtc3(&Serial2, SERIAL1_TX, SERIAL1_RX, MODBUS_DE_PINOUT, MODBUS_RE_PINOUT, 1, SERIAL_MODBUS_BAUD);

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

void TaskSHTC3Read(void *pvParameters) {
  while(1) {
    float humidity = shtc3.getHumidity();
    float temperature = shtc3.getTemperature();

    if (humidity != 0 && temperature != 0) {
      Serial.print("SHTC3 Humidity: ");
      Serial.print(humidity);
      Serial.println("%");
      Serial.print("SHTC3 Temperature: ");
      Serial.print(temperature);
      Serial.println(" °C");

      tb.sendTelemetryData("humidity", humidity);
      tb.sendTelemetryData("temperature", temperature);
    } else {
      Serial.println("Failed to read from SHTC3 sensor!");
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
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
  
  xTaskCreate(TaskSHTC3Read, "SHTC3 Read Task", 2048, NULL, 1, NULL);
  xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCheckTBConnection, "Check Thingsboard connection", 2048, NULL, 2, NULL);
  // xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
}

void loop() {
}
