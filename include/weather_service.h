#ifndef WEATHER_SERVICE_H
#define WEATHER_SERVICE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>
#include <ThingsBoard.h>

class WeatherService {
public:
    WeatherService(const char* apiKey, float latitude, float longitude);
    bool getCurrentWeather();
    float getTemperature() const { return temperature; }
    float getFeelsLike() const { return feelsLike; }  // Added getter for feels_like
    float getPressure() const { return pressure; }    // Added getter for pressure
    float getHumidity() const { return humidity; }
    float getWindSpeed() const { return windSpeed; }
    float getWindGust() const { return windGust; }    // Added getter for wind gust
    float getRain() const { return rain1h; }          // Changed to 1h rain data
    float getClouds() const { return clouds; }
    void sendWeatherTelemetry(ThingsBoard& tb);

private:
    static constexpr const char* WEATHER_HOST = "api.openweathermap.org";
    static constexpr int WEATHER_PORT = 80;
    static constexpr size_t JSON_BUFFER_SIZE = 2048;
    
    const char* apiKey;
    float latitude;
    float longitude;
    
    float temperature;
    float feelsLike;      // Added for feels_like
    float pressure;       // Added for pressure
    float humidity;
    float windSpeed;
    float windGust;       // Added for wind gust
    float rain1h;         // Changed to 1h rain data
    float clouds;
    int visibility;       // Added for visibility
    
    WiFiClient wifiClient;
    HttpClient client;
    
    bool parseWeatherJson(String& json);
};

#endif // WEATHER_SERVICE_H