#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"


#define SPI_SPEED SD_SCK_MHZ[50]12

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
using namespace BLA;

Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);
imu::Vector<3> accldata, gyrodata;

SdFat sd;
SdFile myFile;

// change this to match your SD shield or module;
const int chipSelect = 10;
bool detectError;
bool landed;
int loops = 338;
char line[10];
bool transmit = false;
int timestamptracker = 0;
int TimeStamps[1000]; // check if needs to be higher
float frequency;
byte headercount = 1; //should start at 1 for "Acceleration/Gyroscope" header, increment by 1 for every "No BNO055 detected" written

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  Serial.print("Initializing SD card...");
  sd.begin(chipSelect, SPI_HALF_SPEED);
  Serial.println("initialization done.");

  myFile.open("FullScale.csv", O_RDWR | O_CREAT | O_AT_END);
  myFile.println("Acceleration/Gyroscope");
  myFile.close();
  detectError = bno.begin();
  delay(100);
  while(detectError == 0)
  {
    detectError = bno.begin();
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
    myFile.open("FullScale.csv", O_RDWR | O_AT_END);
    myFile.println("No BNO055 detected");
    myFile.close();
    headercount++;
    delay(100);
  }
  landed = true;
  curtime[0] = 0;
}

void loop()
{
  if(landed == false)
  {
    myFile.open("FullScale.csv", O_RDWR | O_AT_END);
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
    loops++;
  }
}
