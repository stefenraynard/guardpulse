#ifndef AGENT_SENSORS_MAX30102_H
#define AGENT_SENSORS_MAX30102_H

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h" // SparkFun library supports MAX30102

// SENS-004 & SENS-005: Validation ranges
#define MIN_VALID_BPM 40.0
#define MAX_VALID_BPM 220.0
#define MIN_VALID_SPO2 70.0
#define MAX_VALID_SPO2 100.0

// Sentinel values for invalid/stale data (SENS-006)
#define INVALID_BPM 0.0
#define INVALID_SPO2 0.0

class Max30102Agent {
public:
    Max30102Agent();
    
    // Initialize the sensor (requires I2C mutex to be taken by caller if shared)
    bool begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD);
    
    // Read BPM. Returns INVALID_BPM if out of range or no beat detected.
    float getBPM();
    
    // Read SpO2. Returns INVALID_SPO2 if out of range or not detected.
    float getSpO2();

    // Get the last read IR value for debugging
    long getLastIR();

private:
    MAX30105 particleSensor;
    float lastBpm;
    uint32_t lastBeatTime;
    long lastIrValue;
    
    // SpO2 calculation variables
    float lastSpo2;
    uint32_t irMin, irMax;
    uint32_t redMin, redMax;
    uint32_t spo2SampleCount;
    
    // Averaging array for BPM stability
    static const byte RATE_SIZE = 4;
    float rates[RATE_SIZE];
    byte rateSpot;
};

#endif // AGENT_SENSORS_MAX30102_H
