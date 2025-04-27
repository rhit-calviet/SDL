#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "BluetoothSerial.h"
#include "mbedtls/aes.h"  // For AES encryption

// Create objects
MAX30105 particleSensor;
BluetoothSerial SerialBT;

// ESP32 I2C Pins
#define SDA_PIN 21
#define SCL_PIN 22

// Two 256-bit keys (32 bytes each) for "AES-512" emulation
const byte aesKey1[32] = {
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
  0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,
  0xF0, 0xE1, 0xD2, 0xC3, 0xB4, 0xA5, 0x96, 0x87,
  0x78, 0x69, 0x5A, 0x4B, 0x3C, 0x2D, 0x1E, 0x0F
};

const byte aesKey2[32] = {
  0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88,
  0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0x00,
  0x0F, 0x1E, 0x2D, 0x3C, 0x4B, 0x5A, 0x69, 0x78,
  0x87, 0x96, 0xA5, 0xB4, 0xC3, 0xD2, 0xE1, 0xF0
};

// Function to perform "AES-512" style encryption
void encryptAES512(const byte* input, byte* output) {
  mbedtls_aes_context aes1, aes2;
  mbedtls_aes_init(&aes1);
  mbedtls_aes_init(&aes2);

  // Temporary buffer
  byte intermediate[16];

  // First encryption with Key1 (AES-256)
  mbedtls_aes_setkey_enc(&aes1, aesKey1, 256);
  mbedtls_aes_crypt_ecb(&aes1, MBEDTLS_AES_ENCRYPT, input, intermediate);

  // Second encryption with Key2 (AES-256)
  mbedtls_aes_setkey_enc(&aes2, aesKey2, 256);
  mbedtls_aes_crypt_ecb(&aes2, MBEDTLS_AES_ENCRYPT, intermediate, output);

  mbedtls_aes_free(&aes1);
  mbedtls_aes_free(&aes2);
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
  Serial.begin(115200);
  SerialBT.begin("ESP32_AES512");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  particleSensor.setup(60, 4, 2, 100, 411, 4096);

  Serial.println(F("ESP32 AES512 Classic BT Transmission started."));
}

void loop() {
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

  String data = "HR=" + String(heartRate) +
                ", HRvalid=" + String(validHeartRate) +
                ", SPO2=" + String(spo2) +
                ", SPO2Valid=" + String(validSPO2) +
                ", Temp=" + String(temperature);

  // Convert to byte array
  byte dataBytes[data.length() + 1];
  data.getBytes(dataBytes, data.length() + 1);

  // Perform "AES-512" encryption
  byte encryptedData[16]; // Encrypting 16 bytes block
  encryptAES512(dataBytes, encryptedData);

  // Send encrypted data over Bluetooth
  SerialBT.write(encryptedData, sizeof(encryptedData));

  Serial.println(data);

  delay(2000);
}
