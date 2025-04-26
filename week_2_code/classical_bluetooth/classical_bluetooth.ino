/*
  MAX30105 Sensor Data Transmission via Classic Bluetooth
  --------------------------------------------------------
  This code reads Heart Rate, SPO2, and Temperature data
  from the MAX30105 sensor and sends it over Classic Bluetooth (SerialBT).
  
  Hardware:
  - ESP32
  - MAX30105 sensor
  - Arduino IDE with required libraries:
    * SparkFun MAX3010x Sensor Library
    * BluetoothSerial
*/

#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"

// Create sensor and Bluetooth objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

#define MAX_BRIGHTNESS 255

// Define I2C pins for ESP32
#define SDA_PIN 21
#define SCL_PIN 22

// Define pin for LED blink indication
const byte readLED = 4;

// Buffers for sensor data
uint32_t irBuffer[100];
uint32_t redBuffer[100];

// Variables for sensor readings
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup() {
  Serial.begin(115200);          // Start serial monitor
  SerialBT.begin("ESP32_BT");    // Start Classic Bluetooth with device name "ESP32_BT"
  
  pinMode(readLED, OUTPUT);
  Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Check wiring or power."));
    while (1); // Halt program
  }

  Serial.println(F("Attach sensor to finger. Press any key to start."));
  while (Serial.available() == 0);
  Serial.read();  // Wait for user input

  // Configure the sensor
  particleSensor.setup(60, 4, 2, 100, 411, 4096);
}

void loop() {
  bufferLength = 100; // Collect 100 samples
  
  // Fill initial data buffers
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calculate initial heart rate and SPO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Continuously collect new samples and transmit
  while (true) {
    // Shift old data
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    // Load new samples
    for (byte i = 75; i < 100; i++) {
      while (!particleSensor.available())
        particleSensor.check();
      
      digitalWrite(readLED, !digitalRead(readLED)); // Blink LED

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

      Serial.println(data);       // Print to Serial Monitor
      SerialBT.println(data);     // Send via Bluetooth Serial
    }

    // Recalculate heart rate and SPO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
