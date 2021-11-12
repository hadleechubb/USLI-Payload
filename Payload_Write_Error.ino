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
//#include "Adafruit_BMP3XX.h"

//Accelerometer Libraries:
#include <GY_61.h>

//IMU Set UP
uint16_t BNO055_SAMPLERATE_DELAY_MS = .5;
Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);

//SD Card Module Setup
const int chipSelect = 10;

//Declare files for data from each sensor
File IMUAData; // int, 2 bytes //2000 Hz
File IMUGData; // int, 2 bytes //2000 Hz
File AcclData;  // float, 4 bytes //250 Hz (set bandwidth)
//File GPSData;  // // 4000 Hz but saving every few data points
//File AltData; // unsigned int 8, 2 bytes //100 Hz

// array to store IMU acceleration data
int IMU_acclArray[60][3];

// array to store gyroscope data
int IMU_gyroArray[60][3];

//Accelorometer Setup
GY_61 accel;
//accel = GY_61(A1, A2, A3);
int AcclArray[60][3];

void setup() {
  
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("Payload_Write Test"); 
  Serial.println("");

  // Display message if IMU not detected
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }
  // Begin SD
  SD.begin();
  IMUGData = SD.open("IMU_G.csv",FILE_WRITE);
  IMUGData.println("End of launch.");
  IMUGData.close();
  IMUAData = SD.open("IMU_A.csv",FILE_WRITE);
  IMUAData.println("End of launch.");
  IMUAData.close();
  AcclData = SD.open("Accl.csv",FILE_WRITE);
  AcclData.println("End of launch.");
  AcclData.close();
//  GPSData = SD.open("GPS.csv",FILE_WRITE);
//  GPSData.println("End of launch.");
//  GPSData.close();
//  AltData = SD.open("Alt.csv",FILE_WRITE);
//  AltData.println("End of launch.");
//  AltData.close();
//  IMUGData = SD.open("IMU_G.csv",FILE_WRITE);
//  IMUGData.println("Beginning of launch:");
//  IMUGData.close();
//  IMUAData = SD.open("IMU_A.csv",FILE_WRITE);
//  IMUAData.println("Beginning of launch:");
//  IMUAData.close();
//  AcclData = SD.open("LAccl.csv",FILE_WRITE);
//  AcclData.println("Beginning of launch:");
//  AcclData.close();
//  GPSData = SD.open("GPS.csv",FILE_WRITE);
//  GPSData.println("Beginning of launch:");
//  GPSData.close();
//  AltData = SD.open("Alt.csv",FILE_WRITE);
//  AltData.println("Beginning of launch:");
//  AltData.close();  
}

void loop() {

  accel = GY_61(A0, A1, A2);
  // Counter for IMU arrays
   word count = 0;
   //begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF_FMC_OFF);
// adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF_FMC_OFF;
   Adafruit_BNO055::adafruit_bno055_opmode_t mode = Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF;
   bno.setMode(mode);
  
//  //Altimeter Setup
//  #define SEALEVELPRESSURE_HPA (1013.25)
//  Adafruit_BMP3XX bmp; // I2C
//  Adafruit_I2CDevice i2c_dev = 0x77;
//  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
//  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
//  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
//  bmp.setOutputDataRate(BMP3_ODR_100_HZ);
//  int AltArray[60];
  
  // declare sensor event variables to store data in temporarily
  sensors_event_t gyrodata,accldata;

  // loop to retrieve data j amount of times
//  for(int j = 0; j < 200000; j++)//200000
//  {
  /******************************   IMU   ****************************/
  
  //    Open Files
  
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
  IMUGData = SD.open("IMU_G.csv", FILE_WRITE);
  IMUGData.println(IMU_gyroArray[count][0]);
  IMUGData.println(IMU_gyroArray[count][1]);
  IMUGData.println(IMU_gyroArray[count][2]);
  IMUGData.close();

  IMUAData = SD.open("IMU_A.csv", FILE_WRITE);
  IMUAData.println(IMU_acclArray[count][0]);
  IMUAData.println(IMU_acclArray[count][1]);
  IMUAData.println(IMU_acclArray[count][2]);
  IMUAData.close();

  /******************************   Accelerometer   ****************************/
  if(count % 8 == 0)
  {
    AcclData = SD.open("LAccl.csv", FILE_WRITE);
    //retrieve acceleration data
    AcclArray[count][0] = (int)accel.readx();
    AcclArray[count][1] = (int)accel.ready();
    AcclArray[count][2] = (int)accel.readz();
    AcclData.println(AcclArray[count][0]);
    AcclData.println(AcclArray[count][1]);
    AcclData.println(AcclArray[count][2]);
    AcclData.close();
  }

  /******************************   Altimeter   ****************************/
//  if(count % 20 == 0)
//  {
//    AltData = SD.open("Alt.csv", FILE_WRITE);
//    AltArray[count] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
//    AltData.println(AltArray[count]);
//    AltData.close();
//  }
  
  /******************************   GPS   ****************************/
  
  
  // increment count
  count++;
  
  // reset count
  if(count == 60) // Change # to max number of loops that can fit in cache
  {
    count = 0;
  }
//  }  
//  Serial.println("Finished.");
}
