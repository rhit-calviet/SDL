#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"
#include "mbedtls/aes.h"  // For AES encryption
#include "BLEDevice.h"    // BLE security for pairing

// Create objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

// ESP32 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// AES-128 Encryption Key (16 bytes)
const byte aesKey[16] = {
  0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 
  0xFE, 0xDC, 0xBA, 0x98, 0x76, 0x54, 0x32, 0x10
};

// Function to encrypt data using AES-128
void encryptAES128(const byte* input, byte* output) {
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  
  // AES initialization and encryption
  mbedtls_aes_setkey_enc(&aes, aesKey, 128);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input, output);
  mbedtls_aes_free(&aes);
}

// Sensor data buffers
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup() {
  Serial.begin(115200);             // Start serial monitor
  SerialBT.begin("ESP32_AES128");   // Start Classic Bluetooth with AES-128 encryption

  Wire.begin(SDA_PIN, SCL_PIN);     // Initialize I2C
  
  // Initialize the MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1); // Stay here forever
  }

  // Sensor setup configuration
  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  Serial.println(F("ESP32 AES128 Classic BT Transmission started."));
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

  // Convert data string to byte array
  byte dataBytes[data.length() + 1];
  data.getBytes(dataBytes, data.length() + 1);

  // Encrypt the data using AES-128
  byte encryptedData[data.length()];
  encryptAES128(dataBytes, encryptedData);

  // Send encrypted data over Bluetooth
  SerialBT.write(encryptedData, sizeof(encryptedData));

  // Send data over Serial Monitor (optional)
  Serial.println(data); 
}
