#include <Wire.h>
#include "MAX30105.h"
#include "BluetoothSerial.h"

// Create objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

// ESP32 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);             // Start serial monitor
  SerialBT.begin("ESP32_IdleMode");  // Start Classic Bluetooth with name

  Wire.begin(SDA_PIN, SCL_PIN);      // Initialize I2C
  
  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1); // Stay here forever
  }

  Serial.println(F("ESP32 Idle Mode started."));
  Serial.println(F("Bluetooth is ON, but no data will be transmitted."));
}

void loop() {
  // Do absolutely nothing except keep Bluetooth alive
  delay(1000);
}
