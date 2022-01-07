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
File IMUAData; // int, 2 bytes //2000 Hz

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
  IMUAData = SD.open("IMU_A.csv",FILE_WRITE);
  IMUAData.println("End of launch.");
  IMUAData.close();

  // Display message if IMU not detected
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
      IMUAData = SD.open("IMU_A.csv",FILE_WRITE);
    IMUAData.println("No BNO055 detected");
    IMUAData.close();
    while(1);
  }
}

void loop() {
  
  // declare sensor event variables to store data in temporarily
  sensors_event_t accldata;
  
  /******************************   IMU   ****************************/
  
  // retrieve linear acceleration data
  //  bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  IMUAData = SD.open("IMU_A.csv", FILE_WRITE);
  
  // put IMU data in arrays  
  IMUAData.println(accldata.acceleration.x);
  IMUAData.println(accldata.acceleration.y);
  IMUAData.println(accldata.acceleration.z);
  
  IMUAData.close();
  
}
