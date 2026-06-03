#include <Arduino.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include "oled.h"
#include "max30102.h"
#include "mpu6050.h"


// Provide the token generation process info
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info
#include "addons/RTDBHelper.h"

#define I2C_SDA 8
#define I2C_SCL 9

// WiFi credentials
#define WIFI_SSID "CEIOT"
#define WIFI_PASSWORD "CE-1OT@!"

// Firebase credentials (from Firebase Console → Project Settings)
#define API_KEY "AIzaSyCfV1SKtW8JPujExcmQbfmMZktto_yBJP4"
#define DATABASE_URL "https://iotmade-default-rtdb.asia-southeast1.firebasedatabase.app"

// Firebase objects
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
OledDisplay oled;

Max30102Agent hrSensor;
Mpu6050Agent fallSensor;

bool firebaseReady = false;
unsigned long sendDataPrevMillis = 0;

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
}

void setupFirebase() {
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    // Anonymous sign-in (or use email/password)
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
    while (!Serial) { delay(3000); }

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(100000);
     if (oled.begin(&Wire)) {
        Serial.println("OLED initialized.");
    } else {
        Serial.println("OLED initialization failed.");
    }    

    // Force MAX30102 to 100kHz to match the bus and prevent the library from resetting it to 400kHz
    if (hrSensor.begin(Wire, 100000)) {
        Serial.println("MAX30102 initialized.");
    } else {
        Serial.println("MAX30102 initialization failed.");
    }

    if (fallSensor.begin(&Wire)) {
        Serial.println("MPU6050 initialized.");
    } else {
        Serial.println("MPU6050 initialization failed.");
    }

    setupWiFi();
    setupFirebase();
}

void loop() {
    // Continuously poll HR sensor
    float bpm = hrSensor.getBPM();
    float spo2 = hrSensor.getSpO2();

    GyroData_t gyroData;
    fallSensor.readData(&gyroData);

    // Send data to Firebase every 2 seconds
    if (firebaseReady && Firebase.ready() && (millis() - sendDataPrevMillis > 2000)) {
        sendDataPrevMillis = millis();

        long irValue = hrSensor.getLastIR();

    //     // Package all data into a single JSON object to avoid blocking the main loop
        FirebaseJson json;
        json.set("bpm", bpm);
        json.set("spo2", spo2);
        // json.set("fall_detected", gyroData.isFall);
        json.set("fall_detected", gyroData.isFall);
        json.set("timestamp", (int)millis());
        
        // Use setJSONAsync directly on Firebase and pass objects by reference
        if (Firebase.setJSONAsync(fbdo, "/health_band", json)) {
            Serial.printf("[Firebase] Async Sent - BPM: %.2f, SpO2: %.2f, Fall: %d\n", bpm, spo2, gyroData.isFall);
        } else {
            Serial.printf("[Firebase Error] %s\n", fbdo.errorReason().c_str());
        }

        // Print Gyro stats for debugging
        Serial.printf("[GYRO DEBUG] Accel: %.2fg | Gyro X: %.2f, Y: %.2f, Z: %.2f | Fall: %d\n", gyroData.accel, gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ, gyroData.isFall);

        if (gyroData.isFall) {
          Serial.println("!!! FALL DETECTED !!!");
          oled.showEmergency();
       }  else if (irValue < 50000) {
            Serial.println("  -> No finger detected.");
            oled.showMessage("Place finger\non sensor...");
        } else {
            // Update OLED with vitals
            oled.showVitals(bpm, spo2);
        }
    }
}
