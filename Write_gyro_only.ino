#include <Wire.h>

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//SD Card Libraries:
#include <SPI.h>
#include <SD.h>

//IMU Set UP
Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);

//Declare files for data from each sensor
File IMUGData; // int, 2 bytes //2000 Hz

void setup() {
  
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("Payload_Write Test"); 
  Serial.println("");

  // Begin SD
  if(!SD.begin(10))
  {
    Serial.print("No SD Card Module detected");
    while(1);
  }
  IMUGData = SD.open("IMU_G.csv",FILE_WRITE);
  IMUGData.println("Beginning of launch.");
  IMUGData.close();

  // Display message if IMU not detected
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
      IMUGData = SD.open("IMU_G.csv",FILE_WRITE);
    IMUGData.println("No BNO055 detected");
    IMUGData.close();
    while(1);
  }
}

void loop() {
    
  // declare sensor event variables to store data in temporarily
  sensors_event_t gyrodata;
  
  /******************************   IMU   ****************************/
  
  // retrieve linear acceleration data
  bno.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  IMUGData = SD.open("IMU_G.csv", FILE_WRITE);
  
  IMUGData.println(gyrodata.gyro.x);
  IMUGData.println(gyrodata.gyro.y);
  IMUGData.println(gyrodata.gyro.z);
  
  IMUGData.close();
  
}
