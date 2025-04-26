#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"

// Create objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

// ESP32 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Sensor data buffers
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup() {
  Serial.begin(115200);               // Start serial monitor
  SerialBT.begin("ESP32_ActiveTx");    // Start Classic Bluetooth

  Wire.begin(SDA_PIN, SCL_PIN);        // Initialize I2C
  
  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1); // Stay here forever
  }

  // Sensor setup configuration
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  Serial.println(F("ESP32 Active Transmission Mode started."));
}

void loop() {
  bufferLength = 100; // Store 100 samples
  
  // Fill the buffers with initial data
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calculate heart rate and SpO2
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2, &validSPO2, &heartRate, &validHeartRate);

  // Read temperature from the sensor
  float temperature = particleSensor.readTemperature();

  // Format data into a string
  String data = "HR=" + String(heartRate) +
                ", HRvalid=" + String(validHeartRate) +
                ", SPO2=" + String(spo2) +
                ", SPO2Valid=" + String(validSPO2) +
                ", Temp=" + String(temperature);

  // Send data over Serial Monitor
  Serial.println(data);

  // Send data over Bluetooth
  SerialBT.println(data);

  // Wait for 2 seconds
  delay(2000);
}
