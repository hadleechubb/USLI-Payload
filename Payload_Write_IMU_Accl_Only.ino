#include <Wire.h>

// IMU Libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//SD Card Libraries:
#include <SPI.h>
#include <SD.h>

//IMU Set UP
uint16_t BNO055_SAMPLERATE_DELAY_MS = .5;
Adafruit_BNO055 bno = Adafruit_BNO055(01, 0x28);

//SD Card Module Setup
const int chipSelect = 10;

//Declare files for data from each sensor
File IMUAData; // int, 2 bytes //2000 Hz

// array to store IMU acceleration data
int IMU_acclArray[60][3];

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
  IMUAData = SD.open("IMU_A.csv",FILE_WRITE);
  IMUAData.println("End of launch.");
  IMUAData.close();
//  Adafruit_BNO055::adafruit_bno055_opmode_t mode = Adafruit_BNO055::OPERATION_MODE_NDOF_FMC_OFF;
//  bno.setMode(mode);

void loop() {

  // Counter for IMU arrays
   word count = 0;
   //begin(adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF_FMC_OFF);
// adafruit_bno055_opmode_t mode = OPERATION_MODE_NDOF_FMC_OFF;
  
  // declare sensor event variables to store data in temporarily
  sensors_event_t accldata;

  // loop to retrieve data j amount of times
//  for(int j = 0; j < 200000; j++)//200000
//  {
  /******************************   IMU   ****************************/

  // retrieve linear acceleration data
  bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_LINEARACCEL);
  
  // put IMU data in arrays  
  IMU_acclArray[count][0] = (int)accldata.acceleration.x;
  IMU_acclArray[count][1] = (int)accldata.acceleration.y;
  IMU_acclArray[count][2] = (int)accldata.acceleration.z;

  // Write data to files
  IMUAData = SD.open("IMU_A.csv", FILE_WRITE);
  IMUAData.println(IMU_acclArray[count][0]);
  IMUAData.println(IMU_acclArray[count][1]);
  IMUAData.println(IMU_acclArray[count][2]);
  IMUAData.close();
  
  // increment count
  count++;
  
  // reset count
  if(count == 60) // Change # to max number of loops that can fit in cache
  {
    count = 0;
  }
//  }  
}
