#include "weather_service.h"

WeatherService::WeatherService(const char* apiKey, float latitude, float longitude) 
    : apiKey(apiKey)
    , latitude(latitude)
    , longitude(longitude)
    , temperature(0)
    , feelsLike(0)
    , pressure(0)
    , humidity(0)
    , windSpeed(0)
    , windGust(0)
    , rain1h(0)
    , clouds(0)
    , visibility(0)
    , client(wifiClient, WEATHER_HOST, WEATHER_PORT) {
}

bool WeatherService::getCurrentWeather() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi not connected, skipping weather update");
        return false;
    }

    String path = "/data/2.5/weather?lat=";
    path += String(latitude, 8);  // 8 decimal places precision
    path += "&lon=";
    path += String(longitude, 8);
    path += "&units=metric&appid=";
    path += apiKey;

    // Add retry mechanism
    int retries = 3;
    while (retries > 0) {
        int err = client.get(path.c_str());
        if (err == 0) {
            break;
        }
        Serial.printf("Connection failed! Retrying... (%d attempts left)\n", retries-1);
        delay(1000);
        retries--;
    }

    if (retries == 0) {
        Serial.println("Failed to connect after 3 attempts");
        return false;
    }

    int status = client.responseStatusCode();
    if (status != 200) {
        Serial.printf("HTTP request failed, status: %d\n", status);
        return false;
    }

    String response = client.responseBody();
    return parseWeatherJson(response);
}

bool WeatherService::parseWeatherJson(String& json) {
    StaticJsonDocument<JSON_BUFFER_SIZE> doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        Serial.print("JSON parsing failed! Error: ");
        Serial.println(error.c_str());
        return false;
    }

    // Main weather data
    temperature = doc["main"]["temp"].as<float>();
    feelsLike = doc["main"]["feels_like"].as<float>();
    pressure = doc["main"]["pressure"].as<float>();
    humidity = doc["main"]["humidity"].as<float>();
    
    // Wind data
    windSpeed = doc["wind"]["speed"].as<float>();
    if (doc["wind"].containsKey("gust")) {
        windGust = doc["wind"]["gust"].as<float>();
    } else {
        windGust = 0.0f;
    }
    
    // Rain data - check for 1h rainfall
    if (doc.containsKey("rain") && doc["rain"].containsKey("1h")) {
        rain1h = doc["rain"]["1h"].as<float>();
    } else {
        rain1h = 0.0f;
    }

    // Clouds and visibility
    clouds = doc["clouds"]["all"].as<float>();
    visibility = doc["visibility"].as<int>();
    
    return true;
}

void WeatherService::sendWeatherTelemetry(ThingsBoard& tb) {
    tb.sendTelemetryData("long", longitude);
    tb.sendTelemetryData("lat", latitude);
    tb.sendTelemetryData("weatherTemperature", temperature);
    tb.sendTelemetryData("weatherFeelsLike", feelsLike);
    tb.sendTelemetryData("weatherPressure", pressure);
    tb.sendTelemetryData("weatherHumidity", humidity);
    tb.sendTelemetryData("weatherWindSpeed", windSpeed);
    tb.sendTelemetryData("weatherWindGust", windGust);
    tb.sendTelemetryData("weatherRain1h", rain1h);
    tb.sendTelemetryData("weatherClouds", clouds);
    tb.sendTelemetryData("weatherVisibility", visibility);
}