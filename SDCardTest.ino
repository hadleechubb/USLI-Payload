#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define SPI_SPEED SD_SCK_MHZ(50)

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);
sensors_event_t accldata, gyrodata;

SdFat sd;
SdFile myFile;

// change this to match your SD shield or module;
const int chipSelect = 9;
int detectError;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

//  if (!SD.begin(10)) {
//    Serial.println("initialization failed!");
//    return;
//  }
  Serial.println("initialization done.");
  sd.begin(chipSelect, SPI_HALF_SPEED);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile.open("SDTest.csv", O_RDWR | O_CREAT | O_AT_END);
  myFile.close();
  detectError = bno.begin();
  delay(100);
  while(detectError == 0)
  {
    detectError = bno.begin();
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
    myFile.open("SDTest.csv",O_RDWR | O_AT_END);
    myFile.println("No BNO055 detected");
    myFile.close();
    delay(100);
  }
}

void loop()
{
  myFile.open("SDTest.csv",O_RDWR | O_AT_END);
  myFile.println(millis());
  for(int x = 0; x < 2000; x++)
  {
    bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    myFile.println(accldata.acceleration.x);
    myFile.println(accldata.acceleration.y);
    myFile.println(accldata.acceleration.z);
    bno.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
    myFile.println(gyrodata.gyro.x);
    myFile.println(gyrodata.gyro.y);
    myFile.println(gyrodata.gyro.z);
  }
  myFile.println(millis());
  myFile.close();
}
