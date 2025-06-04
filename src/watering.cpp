#include "watering.h"

WateringSystem::WateringSystem(float soilHumidityTarget) {
    this->soilHumidityTarget = soilHumidityTarget;
    
    // Initialize coefficients
    this->soilHumidityCoefficient_P = 1;
    this->soilHumidityCoefficient_I = 1;
    this->soilHumidityCoefficient_Sum = PUMP_POWER_MIN;
    
    // Initialize flow rate tracking
    this->startTime = 0;
    this->totalFlowRate = 0;
    this->flowRateCount = 0;
    this->wateringState = false;
    this->flowRate = 0;

    this->pumpOff();
}

void WateringSystem::flowRateControl(float soilHumidity, HumidityGroupStats stats, float& expectedTime) {
    int groupIndex = static_cast<int>(soilHumidity / 10.0f);
    
    if (stats.hasData && stats.samples >= WATERING_SAMPLE) {
        this->flowRate = PUMP_POWER_MAX; // Use max flow rate if initial humidity data is available
        // Calculate expected time based on average flow rate and time
        expectedTime = stats.avgTime * (stats.avgFlowRate / PUMP_POWER_MAX);
        Serial.printf("Using historical data - Est. time: %.1fs for group %d%%-%d%%\n", 
                     expectedTime, groupIndex * 10, (groupIndex + 1) * 10);
        return;
    }

    // Fall back to PI control if no historical data or not enough samples
    this->soilHumidityCoefficient_Sum += (this->soilHumidityTarget - BRAKE_COEFFICIENT - soilHumidity);
    this->flowRate = (this->soilHumidityTarget - soilHumidity) * this->soilHumidityCoefficient_P
                    + this->soilHumidityCoefficient_Sum * this->soilHumidityCoefficient_I;
    
    if (this->flowRate > PUMP_POWER_MAX) {
        this->flowRate = PUMP_POWER_MAX;
    } else if (this->flowRate < PUMP_POWER_MIN) {
        this->flowRate = PUMP_POWER_MIN;
    }

    Serial.println("Using PI control (insufficient historical data)");
}

void WateringSystem::watering(float soilHumidity, HumidityGroupStats stats) {
    float expectedTime = 0;
    this->flowRateControl(soilHumidity, stats, expectedTime);
    this->soilHumidity = soilHumidity;

    if (this->soilHumidity < this->soilHumidityTarget) {
        if (!this->wateringState) {
            // Start new watering cycle
            this->startTime = millis();
            this->totalFlowRate = 0;
            this->flowRateCount = 0;
        }
        
        if (stats.hasData && stats.samples >= WATERING_SAMPLE) {
            // Check if we're still within the expected watering time
            float elapsedTime = (millis() - this->startTime) / 1000.0f;
            if (elapsedTime < expectedTime) {
                this->wateringState = true;
                this->usingHistoricalControl = true;
                this->pumpOn(this->flowRate);
            } else {
                // If we exceeded expected time, stop watering
                this->wateringState = false;
                this->usingHistoricalControl = false;
                this->soilHumidityCoefficient_Sum = 0;
                this->pumpOff();
            }
        } else {
            // No historical data or not enough samples, use normal PI control
            this->wateringState = true;
            this->usingHistoricalControl = false;
            this->pumpOn(this->flowRate);
        }
        
        // Track flow rate
        this->totalFlowRate += this->flowRate;
        this->flowRateCount++;
    } else {
        this->wateringState = false;
        this->usingHistoricalControl = false;
        this->soilHumidityCoefficient_Sum = PUMP_POWER_MIN;
        this->pumpOff();
    }
}

float WateringSystem::getAverageFlowRate() const {
    return flowRateCount > 0 ? totalFlowRate / flowRateCount : 0;
}

unsigned long WateringSystem::getWateringDuration() const {
    return wateringState ? (millis() - startTime) : (startTime > 0 ? millis() - startTime : 0);
}

void WateringSystem::pumpOn(float flowRate) {
    // Simulate pump on with the given flow rate
    Serial.print("Pump is ON with flow rate: ");
    Serial.println(flowRate);
    analogWrite(PUMP_PIN, flowRate);
}

void WateringSystem::pumpOff() {
    // Simulate pump off
    Serial.println("Pump is OFF");
    analogWrite(PUMP_PIN, 0);
    float avgFlowRate = this->getAverageFlowRate();
    Serial.printf("Average flow rate: %.2f ml/s\n", avgFlowRate);
}