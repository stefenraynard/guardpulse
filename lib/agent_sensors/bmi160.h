#ifndef BMI160_AGENT_H
#define BMI160_AGENT_H

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>

#ifndef GYRO_DATA_T
#define GYRO_DATA_T
struct GyroData_t {
    float accel;
    float gyroX;
    float gyroY;
    float gyroZ;
    bool isFall;
};
#endif

class Bmi160Agent {
public:
    Bmi160Agent();
    bool begin(TwoWire *wire = &Wire);
    void readData(GyroData_t *data);
    bool isFall(GyroData_t *data);

private:
    DFRobot_BMI160 bmi160;
    bool freeFallDetected;
    uint32_t freeFallTime;
};

#endif