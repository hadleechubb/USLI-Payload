#include <Wire.h>

//SD Card Libraries:
#include <SPI.h>
#include <SD.h>

// Declare all Files
File IMUAcclData; // int, 2 bytes //2000 Hz
File IMUGyroData; // int, 2 bytes //2000 Hz
File AcclData;  // float, 4 bytes //250 Hz (set bandwidth)
File GPSData;  // // 4000 Hz but saving every few data points
File AltData; // unsigned int 8, 2 bytes //100 Hz

// change this to match your SD shield or module;
const int chipSelect = 10;

void setup() {
  Serial.begin(9600);
  IMUAcclData = SD.open("IMUAcclData.txt");
  if (IMUAcclData) {
    Serial.println("IMU Acceleration Data:");
  }
  while (IMUAcclData.available()) {
    Serial.write(IMUAcclData.read());
  }
  IMUAcclData.close();
  
  IMUGyroData = SD.open("IMUGyroData.txt");
  if (IMUGyroData) {
    Serial.println("IMU Gyroscope Data:");
  }
  while (IMUGyroData.available()) {
    Serial.write(IMUGyroData.read());
  }
  IMUGyroData.close();

  AcclData = SD.open("AcclData.txt");
  if (AcclData) {
    Serial.println("Accelerometer Data:");
  }
  while (AcclData.available()) {
    Serial.write(AcclData.read());
  }
  AcclData.close();
  
}

void loop() {
  // Nothing happens after setup
}
