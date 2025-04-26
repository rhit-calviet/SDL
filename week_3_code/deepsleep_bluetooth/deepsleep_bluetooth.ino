#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"
#include "esp_sleep.h" // Deep sleep functions

// Create objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

// ESP32 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Deep Sleep Duration (in microseconds)
#define SLEEP_DURATION_US 30e6 // 30 seconds

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
  delay(500);                         // Give time for serial monitor to connect

  SerialBT.begin("ESP32_DeepSleepTx"); // Start Classic Bluetooth

  Wire.begin(SDA_PIN, SCL_PIN);        // Initialize I2C

  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1); // Stay here forever
  }

  // Sensor setup configuration
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  Serial.println(F("ESP32 woke up from Deep Sleep!"));
  
  // Read sensor data
  bufferLength = 100;
  
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();
      
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer,
                                         &spo2, &validSPO2, &heartRate, &validHeartRate);

  float temperature = particleSensor.readTemperature();

  // Prepare data string
  String data = "HR=" + String(heartRate) +
                ", HRvalid=" + String(validHeartRate) +
                ", SPO2=" + String(spo2) +
                ", SPO2Valid=" + String(validSPO2) +
                ", Temp=" + String(temperature);

  // Send data over Serial Monitor
  Serial.println(data);

  // Send data over Bluetooth
  SerialBT.println(data);

  delay(500); // Give Bluetooth a moment to send

  // Prepare to go back to sleep
  Serial.println(F("Going to deep sleep for 30 seconds..."));
  SerialBT.println(F("Going to deep sleep..."));
  delay(100);

  SerialBT.end(); // Stop Bluetooth before sleep (important)

  // Set up timer to wake up ESP32 after SLEEP_DURATION_US microseconds
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
  esp_deep_sleep_start();
}

void loop() {
  // Nothing to do here, everything happens in setup()
}
