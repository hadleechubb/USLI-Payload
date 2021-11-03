#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 2.5;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
word count = 0;
float accl[40000][4];
float gyro[40000][4];

void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
}

void loop() {
  sensors_event_t gyrodata,accldata;
  bno.getEvent(&gyrodata, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accldata, Adafruit_BNO055::VECTOR_LINEARACCEL);
  for (int i = 0; i < 5; i++) {
  gyro[count][1] = gyrodata.data[1];
//  accl[count][i] = accldata.data[i];
  }
  count++;
}
