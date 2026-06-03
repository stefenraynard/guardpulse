#ifndef AGENT_SENSORS_MPU6050_H
#define AGENT_SENSORS_MPU6050_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

struct GyroData_t {
    float accel; // Total acceleration magnitude in g
    float gyroX;
    float gyroY;
    float gyroZ;
    bool isFall; // Flag evaluated by checking conditions
};

class Mpu6050Agent {
public:
    Mpu6050Agent();
    
    // Initialize the sensor
    bool begin(TwoWire *wire = &Wire);
    
    // Read sensor data and perform basic fall condition checks (SENS-003)
    void readData(GyroData_t *data);
    
    // Evaluate fall logic based on FALL-001 (Acceleration > 2g, sudden change, free-fall < 0g)
    bool isFall(GyroData_t *data);

private:
    Adafruit_MPU6050 mpu;
    
    // Simple state tracking for fall sequence
    bool freeFallDetected;
    uint32_t freeFallTime;
};

#endif // AGENT_SENSORS_MPU6050_H
