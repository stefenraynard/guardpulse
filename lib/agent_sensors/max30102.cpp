#include "max30102.h"
#include "heartRate.h" // from SparkFun library

Max30102Agent::Max30102Agent() : lastBpm(INVALID_BPM), lastBeatTime(0), lastIrValue(0), rateSpot(0),
                                 lastSpo2(INVALID_SPO2), irMin(0xFFFFFFFF), irMax(0), redMin(0xFFFFFFFF), redMax(0), spo2SampleCount(0) {
    for (byte i = 0 ; i < RATE_SIZE ; i++) rates[i] = 0;
}

bool Max30102Agent::begin(TwoWire &wirePort, uint32_t i2cSpeed) {
    // SENS-001 & SENS-002: initialization
    if (!particleSensor.begin(wirePort, i2cSpeed)) {
        return false;
    }
    
    // Setup to sense up to 18 inches, max LED brightness
    byte ledBrightness = 60; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //Options: 69, 118, 215, 411
    int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    return true;
}

long Max30102Agent::getLastIR() {
    return lastIrValue;
}

float Max30102Agent::getBPM() {
    long irValue = particleSensor.getIR();
    lastIrValue = irValue;
    
    if (irValue < 50000) {
        lastBpm = INVALID_BPM; // Reset if finger removed
        rateSpot = 0;          // Clear averaging array progress
        for (byte i = 0 ; i < RATE_SIZE ; i++) rates[i] = 0;
        return INVALID_BPM; // No finger detected
    }
    
    if (checkForBeat(irValue) == true) {
        
        uint32_t now = millis();
        if (lastBeatTime != 0) {
            uint32_t delta = now - lastBeatTime;
            float currentBpm = 60000.0 / (float)delta;
            
            if (currentBpm >= MIN_VALID_BPM && currentBpm <= MAX_VALID_BPM) {
                rates[rateSpot++] = currentBpm;
                rateSpot %= RATE_SIZE; // Wrap around to start of array
                
                // Calculate average
                float beatAvg = 0;
                int validRates = 0;
                for (byte x = 0 ; x < RATE_SIZE ; x++) {
                    if (rates[x] > 0) {
                        beatAvg += rates[x];
                        validRates++;
                    }
                }
                if (validRates > 0) {
                    lastBpm = beatAvg / validRates;
                }
            }
        }
        lastBeatTime = now;
    }
    return lastBpm;
}

float Max30102Agent::getSpO2() {
    long irValue = particleSensor.getIR();
    long redValue = particleSensor.getRed();
    
    if (irValue < 50000) {
        lastSpo2 = INVALID_SPO2;
        irMin = 0xFFFFFFFF; irMax = 0;
        redMin = 0xFFFFFFFF; redMax = 0;
        spo2SampleCount = 0;
        return INVALID_SPO2;
    }
    
    // Accumulate min and max for AC/DC calculation
    if (irValue < irMin) irMin = irValue;
    if (irValue > irMax) irMax = irValue;
    if (redValue < redMin) redMin = redValue;
    if (redValue > redMax) redMax = redValue;
    
    spo2SampleCount++;
    
    // Calculate SpO2 roughly every 100 samples (approx 1 second window)
    if (spo2SampleCount >= 100) {
        float irDC = (irMax + irMin) / 2.0;
        float irAC = (irMax - irMin);
        float redDC = (redMax + redMin) / 2.0;
        float redAC = (redMax - redMin);
        
        if (irDC > 0 && redDC > 0 && irAC > 0) {
            float ratio = (redAC / redDC) / (irAC / irDC);
            float currentSpo2 = 104.0 - 17.0 * ratio; // Standard empirical formula
            
            if (currentSpo2 > 100.0) currentSpo2 = 100.0;
            if (currentSpo2 >= MIN_VALID_SPO2 && currentSpo2 <= MAX_VALID_SPO2) {
                lastSpo2 = currentSpo2;
            }
        }
        
        // Reset for the next window
        irMin = 0xFFFFFFFF; irMax = 0;
        redMin = 0xFFFFFFFF; redMax = 0;
        spo2SampleCount = 0;
    }
    
    return lastSpo2;
}
