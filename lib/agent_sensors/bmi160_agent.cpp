#include "bmi160_agent.h"

// Static member initialization
TwoWire* Bmi160Agent::_wire = nullptr;

Bmi160Agent::Bmi160Agent()
    : kfAx(0.001f, 0.03f, 1.0f, 0.0f),
      kfAy(0.001f, 0.03f, 1.0f, 0.0f),
      kfAz(0.001f, 0.03f, 1.0f, 1.0f),
      kfGx(0.01f, 0.1f, 1.0f, 0.0f),
      kfGy(0.01f, 0.1f, 1.0f, 0.0f),
      kfGz(0.01f, 0.1f, 1.0f, 0.0f),
      _isFall(false),
      freeFallTime(0),
      impactDetected(false),
      impactTime(0) {
    memset(&bmi160Dev, 0, sizeof(bmi160Dev));
}

// --- Bosch API I2C bridge callbacks ---

int8_t Bmi160Agent::i2cRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (!_wire) return -1;
    _wire->beginTransmission(dev_addr);
    _wire->write(reg_addr);
    if (_wire->endTransmission(false) != 0) return -1;
    _wire->requestFrom(dev_addr, (uint8_t)len);
    for (uint16_t i = 0; i < len; i++) {
        if (_wire->available()) {
            data[i] = _wire->read();
        } else {
            return -1;
        }
    }
    return 0; // BMI160_OK
}

int8_t Bmi160Agent::i2cWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len) {
    if (!_wire) return -1;
    _wire->beginTransmission(dev_addr);
    _wire->write(reg_addr);
    for (uint16_t i = 0; i < len; i++) {
        _wire->write(data[i]);
    }
    return (_wire->endTransmission() == 0) ? 0 : -1;
}

void Bmi160Agent::delayMs(uint32_t period) {
    delay(period);
}

// --- Public API ---

bool Bmi160Agent::begin(TwoWire *wire, uint8_t i2cAddr) {
    _wire = wire;

    // Configure device handle for I2C
    bmi160Dev.id = i2cAddr;  // BMI160 I2C address (default: 0x69, SDO HIGH)
    bmi160Dev.intf = BMI160_I2C_INTF;
    bmi160Dev.read = &Bmi160Agent::i2cRead;
    bmi160Dev.write = &Bmi160Agent::i2cWrite;
    bmi160Dev.delay_ms = &Bmi160Agent::delayMs;

    // Initialize sensor — reads chip ID and performs soft reset
    int8_t rslt = bmi160_init(&bmi160Dev);
    if (rslt != BMI160_OK) {
        Serial.printf("[BMI160] Init failed: %d\n", rslt);
        return false;
    }

    // Configure accelerometer: ±2g, 100Hz, normal power
    bmi160Dev.accel_cfg.odr = BMI160_ACCEL_ODR_100HZ;
    bmi160Dev.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi160Dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    bmi160Dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Configure gyroscope: ±2000 dps, 100Hz, normal power
    bmi160Dev.gyro_cfg.odr = BMI160_GYRO_ODR_100HZ;
    bmi160Dev.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    bmi160Dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    bmi160Dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;

    // Apply configuration
    rslt = bmi160_set_sens_conf(&bmi160Dev);
    if (rslt != BMI160_OK) {
        Serial.printf("[BMI160] Config failed: %d\n", rslt);
        return false;
    }

    return true;
}

void Bmi160Agent::readData(GyroData_t *data) {
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    int8_t rslt = bmi160_get_sensor_data(
        (BMI160_ACCEL_SEL | BMI160_GYRO_SEL),
        &accel, &gyro, &bmi160Dev
    );

    if (rslt != BMI160_OK) {
        data->ax = 0;
        data->ay = 0;
        data->az = 0;
        data->gx = 0;
        data->gy = 0;
        data->gz = 0;
        data->accel_g = 1.0f;
        data->gyroX_dps = 0.0f;
        data->gyroY_dps = 0.0f;
        data->gyroZ_dps = 0.0f;
        data->isFall = _isFall;
        return;
    }

    // Populate data struct with raw values
    data->ax = accel.x;
    data->ay = accel.y;
    data->az = accel.z;
    data->gx = gyro.x;
    data->gy = gyro.y;
    data->gz = gyro.z;

    // Convert raw values to float units (g and dps)
    float ax_raw_g = (float)accel.x / 16384.0f;
    float ay_raw_g = (float)accel.y / 16384.0f;
    float az_raw_g = (float)accel.z / 16384.0f;

    float gx_raw_dps = (float)gyro.x / 16.4f;
    float gy_raw_dps = (float)gyro.y / 16.4f;
    float gz_raw_dps = (float)gyro.z / 16.4f;

    // Apply Kalman filter smoothing
    float ax_fil = kfAx.update(ax_raw_g);
    float ay_fil = kfAy.update(ay_raw_g);
    float az_fil = kfAz.update(az_raw_g);

    float gx_fil = kfGx.update(gx_raw_dps);
    float gy_fil = kfGy.update(gy_raw_dps);
    float gz_fil = kfGz.update(gz_raw_dps);

    // Filtered magnitude
    float filtered_mag = sqrt(ax_fil * ax_fil + ay_fil * ay_fil + az_fil * az_fil);

    // Populate filtered values
    data->accel_g = filtered_mag;
    data->gyroX_dps = gx_fil;
    data->gyroY_dps = gy_fil;
    data->gyroZ_dps = gz_fil;

    unsigned long now = millis();

    // 1. Free fall detection: filtered_mag < 0.5f (2.0s window)
    if (filtered_mag < 0.5f) {
        freeFallTime = now;
        _isFall = false; // Reset fall alert if we are in freefall again
    }

    // 2. Impact detection: filtered_mag > 1.5f and gyro spike (any gyro axis > 115 dps) within 2.0s of free fall
    if (freeFallTime != 0 && (now - freeFallTime <= 2000)) {
        bool gyroSpike = (fabsf(gx_fil) > 115.0f || fabsf(gy_fil) > 115.0f || fabsf(gz_fil) > 115.0f);
        if (filtered_mag > 1.5f && gyroSpike) {
            impactDetected = true;
            impactTime = now;
            freeFallTime = 0; // Clear freefall time so we don't trigger multiple times
        }
    }

    // 3. Post-fall inactivity check: after impact, monitor gyro for 1.5s. If any gyro axis > 40 dps, cancel fall alert.
    // If it remains still for 1.5s, trigger isFall = true.
    if (impactDetected) {
        if (now - impactTime <= 1500) {
            if (fabsf(gx_fil) > 40.0f || fabsf(gy_fil) > 40.0f || fabsf(gz_fil) > 40.0f) {
                impactDetected = false;
                impactTime = 0;
            }
        } else {
            _isFall = true;
            impactDetected = false;
            impactTime = 0;
        }
    }

    // 4. Recovery: if a fall was triggered, but the person is active again (any gyro axis > 60 dps)
    if (_isFall) {
        if (fabsf(gx_fil) > 60.0f || fabsf(gy_fil) > 60.0f || fabsf(gz_fil) > 60.0f) {
            _isFall = false;
        }
    }

    data->isFall = _isFall;
}

