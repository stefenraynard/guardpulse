#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "oled.h"
#include "max30102_agent.h"
#include "bmi160_agent.h"

// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info
#include "addons/RTDBHelper.h"

#define I2C_SDA 8
#define I2C_SCL 9

// WiFi credentials
#define WIFI_SSID "E948"
#define WIFI_PASSWORD "123456789"

// Firebase credentials (from Firebase Console → Project Settings)
#define API_KEY "AIzaSyCfV1SKtW8JPujExcmQbfmMZktto_yBJP4"
#define DATABASE_URL "https://iotmade-default-rtdb.asia-southeast1.firebasedatabase.app"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
OledDisplay oled;

Max30102Agent hrSensor;
Bmi160Agent fallSensor;

bool firebaseReady = false;
unsigned long sendDataPrevMillis = 0;

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs < 5000)) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("Connected with IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("WiFi connection failed (timeout).");
    }
}

void setupFirebase() {
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    // Anonymous sign-in
    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Firebase sign-up OK");
        firebaseReady = true;
    } else {
        Serial.printf("Firebase sign-up error: %s\n", config.signer.signupError.message.c_str());
    }

    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

void setup() {
    Serial.begin(115200);
    unsigned long serialStart = millis();
    while (!Serial && (millis() - serialStart < 3000)) {
        delay(10);
    }

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
    
    if (oled.begin(&Wire)) {
        Serial.println("OLED initialized.");
    } else {
        Serial.println("OLED initialization failed.");
    }    

    // Initialize MAX30102
    if (hrSensor.begin(Wire, 100000)) {
        Serial.println("MAX30102 initialized.");
    } else {
        Serial.println("MAX30102 initialization failed.");
    }

    // Try BMI160 at address 0x68. If that fails, try address 0x69.
    bool bmiSuccess = fallSensor.begin(&Wire, 0x68);
    if (!bmiSuccess) {
        bmiSuccess = fallSensor.begin(&Wire, 0x69);
    }
    if (bmiSuccess) {
        Serial.println("BMI160 initialized.");
    } else {
        Serial.println("BMI160 initialization failed.");
    }

    setupWiFi();
    if (WiFi.status() == WL_CONNECTED) {
        setupFirebase();
    }
}

void loop() {
    // Continuously read/poll MAX30102 raw values
    uint32_t rawIr = 0, rawRed = 0;
    hrSensor.readRaw(rawIr, rawRed);

    // Read BMI160 filtered and raw data
    GyroData_t gyroData;
    fallSensor.readData(&gyroData);

    // Upload data asynchronously to Firebase every 10 seconds
    if (millis() - sendDataPrevMillis > 10000) {
        sendDataPrevMillis = millis();

        float bpm = hrSensor.getBPM();
        float spo2 = hrSensor.getSpO2();

        // Print debug log
        Serial.printf("[DEBUG] BPM: %.2f, SpO2: %.2f, Fall: %d, Accel: %.2fg, Gyro: %.2f | Raw IR: %u, Raw Red: %u, BufLen: %d\n",
                      bpm, spo2, gyroData.isFall, gyroData.accel_g, gyroData.gyroX_dps, rawIr, rawRed, hrSensor.getBufferLength());

        if (firebaseReady && Firebase.ready()) {
            FirebaseJson json;
            json.set("bpm", bpm);
            json.set("spo2", spo2);
            json.set("fall_detected", gyroData.isFall);
            json.set("timestamp", (int)millis());
            
            if (Firebase.setJSONAsync(fbdo, "/health_band", json)) {
                Serial.println("[Firebase] Async upload started.");
            } else {
                Serial.printf("[Firebase Error] %s\n", fbdo.errorReason().c_str());
            }
        }
    }

    // Update OLED: if fall detected, show emergency screen; else if no finger (IR < 20000), show "Place finger on sensor..."; else, show vitals (BPM, SpO2)
    static unsigned long lastOledMillis = 0;
    if (millis() - lastOledMillis > 500) {
        lastOledMillis = millis();
        if (gyroData.isFall) {
            oled.showEmergency();
        } else if (rawIr < 20000) {
            oled.showMessage("Place finger\non sensor...");
        } else {
            float bpm = hrSensor.getBPM();
            float spo2 = hrSensor.getSpO2();
            oled.showVitals(bpm, spo2);
        }
    }
}
