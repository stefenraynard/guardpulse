#include "mpu6050.h"

Mpu6050Agent::Mpu6050Agent() : freeFallDetected(false), freeFallTime(0) {
}

bool Mpu6050Agent::begin(TwoWire *wire) {
    if (!mpu.begin(0x68, wire)) {
        return false;
    }
    
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    return true;
}

void Mpu6050Agent::readData(GyroData_t *data) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    // Calculate magnitude of acceleration in g (9.81 m/s^2 = 1g)
    float accelX = a.acceleration.x / 9.81;
    float accelY = a.acceleration.y / 9.81;
    float accelZ = a.acceleration.z / 9.81;
    
    data->accel = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);
    data->gyroX = g.gyro.x;
    data->gyroY = g.gyro.y;
    data->gyroZ = g.gyro.z;
    
    data->isFall = isFall(data);
}

bool Mpu6050Agent::isFall(GyroData_t *data) {
    // FALL-001 logic:
    // (a) acceleration > 2g
    // (b) sudden orientation change (gyro spike)
    // (c) acceleration < 0g (or close to 0g, i.e., free-fall phase) within same 2s window
    
    uint32_t now = millis();
    
    // Check for free-fall phase (close to 0g)
    if (data->accel < 0.5) { // Relaxed free-fall threshold for easier triggering
        freeFallDetected = true;
        freeFallTime = now;
    }
    
    // Check if free fall was detected recently (within 2 seconds)
    if (freeFallDetected && (now - freeFallTime > 2000)) {
        freeFallDetected = false; // Reset if too old
    }
    
    // Sudden orientation change (lowered threshold to ~114 degrees/sec)
    bool suddenChange = (abs(data->gyroX) > 2.0 || abs(data->gyroY) > 2.0 || abs(data->gyroZ) > 2.0);
    
    // Final condition
    // Trigger if high impact + tumble, OR if high impact + recent freefall
    if (data->accel > 2.0 && (suddenChange || freeFallDetected)) {
        // Reset state after triggering
        freeFallDetected = false;
        return true;
    }
    
    return false;
}
