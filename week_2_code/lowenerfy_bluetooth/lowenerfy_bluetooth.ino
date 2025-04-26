/*
  MAX30105 Sensor Data Transmission via BLE (Bluetooth Low Energy)
  -----------------------------------------------------------------
  This code reads Heart Rate, SPO2, and Temperature data
  from the MAX30105 sensor and sends it over BLE.

  Hardware:
  - ESP32
  - MAX30105 sensor
  - Arduino IDE with required libraries:
    * SparkFun MAX3010x Sensor Library
    * BLEDevice (for ESP32 BLE)
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
#define SDA_PIN 21
#define SCL_PIN 22

const byte readLED = 4;

uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// BLE settings
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-5678-90ab-cdef-1234567890ab"

BLEServer* pServer = NULL;
BLECharacteristic *pCharacteristic;

void setup() {
  Serial.begin(115200);
  pinMode(readLED, OUTPUT);
  
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Check wiring or power."));
    while (1);
  }

  Serial.println(F("Attach sensor to finger. Press any key to start."));
  while (Serial.available() == 0);
  Serial.read();

  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  // BLE Initialization
  BLEDevice::init("ESP32_BLE");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pService->start();
  pServer->getAdvertising()->start();
}

void loop() {
  bufferLength = 100;

  // Fill initial buffers
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuous sampling and transmission
  while (true) {
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    for (byte i = 75; i < 100; i++) {
      while (!particleSensor.available())
        particleSensor.check();

      digitalWrite(readLED, !digitalRead(readLED));

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
      
      float temperature = particleSensor.readTemperature();

      // Create data string
      String data = "HR=" + String(heartRate) +
                    ", HR_valid=" + String(validHeartRate) +
                    ", SPO2=" + String(spo2) +
                    ", SPO2_valid=" + String(validSPO2) +
                    ", Temp=" + String(temperature);

      Serial.println(data);

      // Send data over BLE
      pCharacteristic->setValue(data.c_str());
      pCharacteristic->notify();
    }

    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
