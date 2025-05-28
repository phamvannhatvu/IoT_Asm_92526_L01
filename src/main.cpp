#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT20.h"
#include "Wire.h"
#include <ArduinoOTA.h>
#include <OTA_Firmware_Update.h>
#include <Espressif_Updater.h>
#include <Shared_Attribute_Update.h>
#include <Attribute_Request.h>

#define SMARTCONFIG_TIMEOUT 30

constexpr char WIFI_SSID[] = "nhatvu";
constexpr char WIFI_PASSWORD[] = "25122003";

constexpr char TOKEN[] = "gd63iniu7du1xm4zetoh";

constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;


constexpr uint16_t WIFI_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_CONNECT_CHECKING_INTERVAL_MS = 10000U;
constexpr uint16_t TB_LOOP_INTERVAL_MS = 10U;
constexpr uint16_t OTA_UPDATE_INTERVAL_MS = 100U;
constexpr uint16_t SEND_TELEMETRY_INTERVAL_MS = 1000U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 9600U;
constexpr uint16_t MAX_MESSAGE_SIZE = 1024U;

// Initalize the Updater client instance used to flash binary to flash memory
Espressif_Updater<> updater;
// Statuses for updating
bool currentFWSent = false;
bool updateRequestSent = false;
// Initialize used apis
OTA_Firmware_Update<> ota;
constexpr uint8_t MAX_ATTRIBUTES = 2U;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
const std::array<IAPI_Implementation*, 3U> apis = {
    &shared_update,
    &attr_request,
    &ota
};

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
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

DHT20 dht20;

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

void TaskTBLoop(void *pvParameters) {
  while(1) {
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(TB_LOOP_INTERVAL_MS));
  }
}

void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  InitWiFi();

  Wire.begin();
  dht20.begin();
  
  xTaskCreate(TaskCheckWiFiConnection, "Check WiFi connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskCheckTBConnection, "Check Thingsboard connection", 2048, NULL, 2, NULL);
  xTaskCreate(TaskTBLoop, "TB Loop", 4096, NULL, 2, NULL);
  xTaskCreate(TaskOTAUpdate, "OTA Update", 4096, NULL, 2, NULL);
  xTaskCreate(TaskReadAndSendTelemetryData, "Read and send telemetry data", 2048, NULL, 2, NULL);
}

void loop() {
  
}
