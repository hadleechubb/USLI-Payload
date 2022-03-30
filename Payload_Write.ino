#include <Wire.h>

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//SD Card Libraries:
#include <SPI.h>
#include <SD.h>

//GPS Libraries:

//Altimeter Libraries:

//Accelerometer Libraries:
#include <GY_61.h>


//Set CS pin for SD Card Module
const int chipSelect = 10;

//Declare files for data from each sensor
File IMUAcclData; // int, 2 bytes //2000 Hz
File IMUGyroData; // int, 2 bytes //2000 Hz
File AcclData;  // float, 4 bytes //250 Hz (set bandwidth)
File GPSData;  // // 4000 Hz but saving every few data points
File AltData; // unsigned int 8, 2 bytes //100 Hz


//IMU Sample Period

uint16_t BNO055_SAMPLERATE_DELAY_MS = .05;

// declare IMU in library
// Check I2C device address and correct line below (address is 0x28 on Arduino Due)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);

// Counter for IMU arrays
word count = 0;

// array to store IMU acceleration data
int IMU_acclArray[40][3];
// todo: figure out the correct size of these

// array to store gyroscope data
int IMU_gyroArray[40][3];

//Accelorometer declaration
GY_61 Accl;
accel = GY_61(A1, A2, A3);

// Acceleration Array
float Accl[40][3];

void setup() {
  
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("Payload_Write Test"); Serial.println("");

  // Display message if IMU not detected
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  //    return;
  }
  // Display message if SD card not detected
  if (!SD.begin()) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
}

void loop() {
  
  // declare sensor event variables to store data in temporarily
  sensors_event_t gyrodata,accldata;

  // loop to retrieve data j amount of times
  for(int j = 0; j < 200000; j++)
  {
    /******************************   IMU   ****************************/
    
    //Open Files
    IMUAcclData = SD.open("IMUAcclData.txt", FILE_WRITE);
    IMUGyroData = SD.open("IMUGyroData.txt", FILE_WRITE);
    
    // retrieve gyroscope data
    bno.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // retrieve linear acceleration data
    bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // todo: figure out when to take other data
    
    // put IMU data in arrays
    IMU_gyroArray[count][0] = (int)gyrodata.gyro.x;
    IMU_gyroArray[count][1] = (int)gyrodata.gyro.y;
    IMU_gyroArray[count][2] = (int)gyrodata.gyro.z;
    
    IMU_acclArray[count][0] = (int)accldata.acceleration.x;
    IMU_acclArray[count][1] = (int)accldata.acceleration.y;
    IMU_acclArray[count][2] = (int)accldata.acceleration.z;

    // Write data to files
    IMUGyroData.println(IMU_gyroArray[count]);
    IMUAcclData.println(IMU_acclArray[count[);

    // Close IMU Files
    IMUGyroData.close();
    IMUAcclData.close();

    /******************************   Accelerometer   ****************************/
    if(count % 8 == 0)
    {
      AcclData = SD.open("AcclData.txt", FILE_WRITE);
      //retrieve acceleration data
      Accl[count][1] = accel.readx();
      Accl[count][2] = accel.ready();
      Accl[count][3] = accel.readz();
      
      AcclData.println(Accl[count]);
      AcclData.close();
    }

    /******************************   Altimeter   ****************************/
    if(count % 20 == 0)
    {
      AltData = SD.open("AltData.txt", FILE_WRITE);
      // retrieve altimeter data
      AltData.close():
    }
    
    /******************************   GPS   ****************************/
    
    
    // increment count
    count++;
    
    // reset count
    if(count > 40) // Change # to max number of loops that can fit in cache
    {
      count = 0;
    }
  }
  
//  Serial.println("Acceleration Data:");
//  for(int k = 0; k < 40; k++)
//  {
//    for(int l = 0; l < 3; l++)
//    {
//      Serial.println(IMU_acclArray[k][l]);
//    }
//  }
//  Serial.println("Gyroscope Data:");
//  for(int k = 0; k < 40; k++)
//  {
//    for(int l = 0; l < 3; l++)
//    {
//      Serial.println(IMU_gyroArray[k][l]);
//    }
//  }
}
//  void setMode(adafruit_bno055_opmode_t mode);
