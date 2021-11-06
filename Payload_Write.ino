#include <Wire.h>
// IMU libraries:
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// library to write to flash:
#include <avr/pgmspace.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 2.5;

// declare IMU in library
// Check I2C device address and correct line below (address is 0x28 on Arduino Due)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Counter for arrays
int count = 0;

// array to store acceleration data in non-volatile flash memory
PROGMEM float acclArray[400][3]; //40000

// array to store gyroscope data in non-volatile flash memory
PROGMEM float gyroArray[400][3];

void setup() {
  // initialize serial communication
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  // display message if sensor not detected
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
}

void loop() {
  // Write
  // declare sensor event variables to store data in temporarily
  sensors_event_t gyrodata,accldata;
  
  // loop to retrieve data j amount of times
  for(int j = 0; j < 40; j++)
  {
    // retrieve gyroscope data
    bno.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
    
    // retrieve linear acceleration data
    bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_LINEARACCEL);
    
    // put data in flash array
    gyroArray[count][1] = gyrodata.gyro.x;
    gyroArray[count][2] = gyrodata.gyro.y;
    gyroArray[count][3] = gyrodata.gyro.z;
    acclArray[count][1] = accldata.acceleration.x;
    acclArray[count][2] = accldata.acceleration.y;
    acclArray[count][3] = accldata.acceleration.z;
    
    // increment count
    count++;
  }
  
  // Read
  // loop through arrays and display data on serial monitor
  for(int k = 0; k < 40; k++)
  {
    for(int l = 0; l < 3; l++)
    {
      Serial.println(acclArray[k][l]);
    }
  }
}
