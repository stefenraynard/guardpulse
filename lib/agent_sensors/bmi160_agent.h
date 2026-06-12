#ifndef BMI160_AGENT_H
#define BMI160_AGENT_H

#include <Arduino.h>
#include <Wire.h>

// Bosch BMI160 API is pure C — wrap for C++ linkage
extern "C" {
#include "bmi160.h"
}

class KalmanFilter {
public:
    KalmanFilter(float q = 0.01f, float r = 0.1f, float p = 1.0f, float initial_value = 0.0f) {
        Q = q;
        R = r;
        P = p;
        x = initial_value;
    }

    float update(float measurement) {
        P = P + Q;
        float K = P / (P + R);
        x = x + K * (measurement - x);
        P = (1.0f - K) * P;
        return x;
    }

private:
    float Q;
    float R;
    float P;
    float x;
};

struct GyroData_t {
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
    float accel_g;
    float gyroX_dps;
    float gyroY_dps;
    float gyroZ_dps;
    bool isFall;
};

class Bmi160Agent {
public:
    Bmi160Agent();
    bool begin(TwoWire *wire = &Wire, uint8_t i2cAddr = 0x69);
    void readData(GyroData_t *data);

private:
    struct bmi160_dev bmi160Dev;

    // Static I2C bridge callbacks for Bosch API
    static TwoWire *_wire;
    static int8_t i2cRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static int8_t i2cWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
    static void delayMs(uint32_t period);

    // Kalman Filter instances
    KalmanFilter kfAx, kfAy, kfAz;
    KalmanFilter kfGx, kfGy, kfGz;

    // Fall detection states
    bool _isFall;
    unsigned long freeFallTime;
    bool impactDetected;
    unsigned long impactTime;
};

#endif

