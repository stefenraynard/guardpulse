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
Bmi160Agent fallSensor;

bool firebaseReady = false;
unsigned long sendDataPrevMillis = 0;

// Device identity variables
String deviceUID = "";
String pairingCode = "";
String ownerUID = "";

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

void checkDeviceStatus() {
    if (WiFi.status() != WL_CONNECTED || !firebaseReady || !Firebase.ready()) {
        return;
    }

    String ownerPath = "/devices/" + deviceUID + "/ownerUID";
    if (Firebase.getString(fbdo, ownerPath)) {
        String val = fbdo.stringData();
        String dataType = fbdo.dataType();
        
        if (dataType == "null" || val.length() == 0) {
            ownerUID = "";
            Serial.println("[Firebase] Registering device...");
            if (Firebase.setString(fbdo, ownerPath, "null") && 
                Firebase.setString(fbdo, "/devices/" + deviceUID + "/pairingCode", pairingCode)) {
                Serial.println("[Firebase] Device registered successfully.");
            } else {
                Serial.printf("[Firebase Error] Registration failed: %s\n", fbdo.errorReason().c_str());
            }
        } else if (val == "null") {
            ownerUID = "";
        } else {
            ownerUID = val;
        }
    } else {
        Serial.printf("[Firebase Error] checkDeviceStatus getString failed: %s\n", fbdo.errorReason().c_str());
        ownerUID = "";
    }
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
    
    String rawMac = WiFi.macAddress();
    rawMac.replace(":", "");
    rawMac.toUpperCase();
    deviceUID = rawMac;
    if (deviceUID.length() >= 6) {
        pairingCode = deviceUID.substring(deviceUID.length() - 6);
    } else {
        pairingCode = deviceUID;
    }
    Serial.print("Device UID: ");
    Serial.println(deviceUID);
    Serial.print("Pairing Code: ");
    Serial.println(pairingCode);

    if (WiFi.status() == WL_CONNECTED) {
        setupFirebase();
        checkDeviceStatus();
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

        checkDeviceStatus();

        float bpm = hrSensor.getBPM();
        float spo2 = hrSensor.getSpO2();

        // Print debug log
        Serial.printf("[DEBUG] BPM: %.2f, SpO2: %.2f, Fall: %d, Accel: %.2fg, Gyro: %.2f | Raw IR: %u, Raw Red: %u, BufLen: %d | Owner: %s\n",
                      bpm, spo2, gyroData.isFall, gyroData.accel_g, gyroData.gyroX_dps, rawIr, rawRed, hrSensor.getBufferLength(), ownerUID.c_str());

        if (firebaseReady && Firebase.ready() && ownerUID != "" && ownerUID != "null") {
            FirebaseJson json;
            json.set("bpm", bpm);
            json.set("spo2", spo2);
            json.set("fall_detected", gyroData.isFall);
            json.set("timestamp", (int)millis());
            
            String uploadPath = "/users/" + ownerUID + "/devices_data/" + deviceUID + "/sensor_data";
            if (Firebase.setJSONAsync(fbdo, uploadPath, json)) {
                Serial.println("[Firebase] Async upload started.");
            } else {
                Serial.printf("[Firebase Error] %s\n", fbdo.errorReason().c_str());
            }
        }
    }

    // Update OLED: if unpaired (ownerUID empty or "null"), show pairing code; else if fall detected, show emergency screen; else if no finger (IR < 20000), show "Place finger on sensor..."; else, show vitals (BPM, SpO2)
    static unsigned long lastOledMillis = 0;
    if (millis() - lastOledMillis > 500) {
        lastOledMillis = millis();
        if (ownerUID == "" || ownerUID == "null") {
            String msg = "Pairing Code:\n" + pairingCode;
            oled.showMessage(msg.c_str());
        } else if (gyroData.isFall) {
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
