#include "bmi160.h"

// Default scale factors for ±2g and ±250 dps ranges
const float scaleFactor = 16384.0;
const float gyroScaleFactor = 131.2;

Bmi160Agent::Bmi160Agent() : freeFallDetected(false), freeFallTime(0) {
}

bool Bmi160Agent::begin(TwoWire *wire) {
    if (bmi160.softReset() != BMI160_OK) {
        return false;
    }
    // Typical I2C address for BMI160 is 0x68 or 0x69
    return bmi160.I2cInit(0x68) == BMI160_OK;
}

void Bmi160Agent::readData(GyroData_t *data) {
    // Read accelerometer and gyroscope data from BMI160
    int16_t accelGyro[6] = {0};
    bmi160.getAccelGyroData(accelGyro);
    
    int16_t gyroX = accelGyro[0];
    int16_t gyroY = accelGyro[1];
    int16_t gyroZ = accelGyro[2];
    int16_t accelX = accelGyro[3];
    int16_t accelY = accelGyro[4];
    int16_t accelZ = accelGyro[5];

    // Convert raw values to standard units (g for accel, degrees/sec for gyro)
    // Note: You must divide by the scale factor corresponding to your configured ranges.
    float aX = accelX / scaleFactor;
    float aY = accelY / scaleFactor;
    float aZ = accelZ / scaleFactor;
    data->accel = sqrt(aX*aX + aY*aY + aZ*aZ);
    
    data->gyroX = gyroX / gyroScaleFactor;
    data->gyroY = gyroY / gyroScaleFactor;
    data->gyroZ = gyroZ / gyroScaleFactor;
    
    data->isFall = isFall(data);
}

bool Bmi160Agent::isFall(GyroData_t *data) {
    // Reusing the same FALL-001 logic
    uint32_t now = millis();
    
    if (data->accel < 0.5) { 
        freeFallDetected = true;
        freeFallTime = now;
    }
    
    if (freeFallDetected && (now - freeFallTime > 2000)) {
        freeFallDetected = false;
    }
    
    bool suddenChange = (abs(data->gyroX) > 2.0 || abs(data->gyroY) > 2.0 || abs(data->gyroZ) > 2.0);
    
    if (data->accel > 2.0 && (suddenChange || freeFallDetected)) {
        freeFallDetected = false;
        return true;
    }
    
    return false;
}