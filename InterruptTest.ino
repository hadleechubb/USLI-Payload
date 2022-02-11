#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define SPI_SPEED SD_SCK_MHZ(50)
#define INTERRUPT_PIN 8
#define RED_LED 5

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);
imu::Vector<3> accldata, gyrodata;

SdFat sd;
SdFile myFile;

// change this to match your SD shield or module;
const int chipSelect = 10;
int detectError;
bool intfired = 0;

void MotionDetected()
{
  intfired = 1; 
}

void setup()
{
  // Open serial communications
  Serial.begin(9600);
  // Initialize SD card
  Serial.print("Initializing SD card...");
  sd.begin(chipSelect, SPI_HALF_SPEED);
  Serial.println("initialization done.");

  // Create File
  myFile.open("SDTest.csv", O_RDWR | O_CREAT | O_AT_END);
  myFile.println("Acceleration/Gyroscope, interrupt status");
  myFile.close();

  // set up LED
  pinMode(RED_LED,OUTPUT);
  digitalWrite(RED_LED, HIGH);

  // set up IMU
  detectError = bno.begin();
  delay(100);
  while(detectError == 0)
  {
    digitalWrite(RED_LED,HIGH);
    detectError = bno.begin();
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
    myFile.open("SDTest.csv", O_RDWR | O_AT_END);
    myFile.println("No BNO055 detected");
    myFile.close();
    delay(100);
  }
  digitalWrite(RED_LED,LOW);

  // set up interrupt
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), MotionDetected, CHANGE); // may need to change mode later
}

void loop()
{
//  myFile.open("SDTest.csv", O_RDWR | O_AT_END);
//  myFile.println(millis());
//  myFile.close();
  while(intfired == 0)
  {
    myFile.open("SDTest.csv", O_RDWR | O_AT_END);
    myFile.println(millis());
    for(int x = 0; x < 1000; x++)
    {
      accldata = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      gyrodata = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      myFile.println(accldata.x());
      myFile.println(accldata.y());
      myFile.println(accldata.z());
      myFile.println(gyrodata.x());
      myFile.println(gyrodata.y());
      myFile.println(gyrodata.z());
    }
    myFile.println(millis());
    myFile.close();
  }
//  myFile.open("SDTest.csv", O_RDWR | O_AT_END);
//  myFile.println(millis());
//  myFile.close();
}
