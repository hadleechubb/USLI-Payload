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

// Function Variables:
int xAcc[1000]; //Initializing acceleration arrays
int yAcc[1000];
int zAcc[1000];

int xAcca[1000]; //Initializing absolute acceleration arrays
int yAcca[1000];
int zAcca[1000];

float acca1[3][3];
float acca2[3][3];
float acca3[3][3];
int acc[3];
float acca[3][3];

int xOme[1000];
int yOme[1000];
int zOme[1000];
float curtime[1000]; 

int xVel[1000]; //Initializing velocity arrays
int yVel[1000];
int zVel[1000];

int xDist[1000]; //Initializing position arrays
int yDist[1000];
int zDist[1000];

int xThe[1000]; //Initializing angle arrays
int yThe[1000];
int zThe[1000];

float apogee;

int finCoords[3];
int xLoc;
int yLoc;
byte cellID;

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

  if(landed == true)
  {
    if(transmit == false)
    {
      if(loops != 0)
      {
        myFile.open("AlgorithmTest.csv", O_RDWR);
        for(int j = 0; j < headercount; j++)
        {
          myFile.fgets(line,sizeof(line));
        }
        // need a way to keep track of where last data was taken from file
        // get beginning time stamp
        myFile.fgets(line,sizeof(line));
        TimeStamps[timestamptracker] = atoi(line);
        timestamptracker++;
        for(int i = 0; i<1000; i++)
        {
          // Accl X
          myFile.fgets(line, sizeof(line));
          xAcc[i] = atoi(line);
          // Accl Y
          myFile.fgets(line, sizeof(line));
          yAcc[i] = atoi(line);
          // Accl Z
          myFile.fgets(line, sizeof(line));
          zAcc[i] = -atoi(line);
          // Gyro X
          myFile.fgets(line, sizeof(line));
          xOme[i] = atoi(line);
          // Gyro Y
          myFile.fgets(line, sizeof(line));
          xOme[i] = atoi(line);
          // Gyro Z
          myFile.fgets(line, sizeof(line));
          xOme[i] = atoi(line);
        }
        // get ending time stamp
        myFile.fgets(line,sizeof(line));
        TimeStamps[timestamptracker] = atoi(line);
        timestamptracker++;
        
        myFile.close();
        frequency = 1000/(TimeStamps[timestamptracker]-TimeStamps[timestamptracker-1]);
        
        finalLoc(frequency, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
      }
    }
  }
}



// location calculation function
int finalLoc(float frequency, float xVel0, float yVel0, float zVel0, float xDist0, float yDist0, float zDist0, float xThe0, float yThe0, float zThe0, float xOff, float yOff)
{    
  xVel[0] = xVel0;
  yVel[0] = yVel0;
  zVel[0] = zVel0;
  
  xDist[0] = xDist0;
  yDist[0] = yDist0;
  zDist[0] = zDist0;   
  
  xThe[0] = xThe0;
  yThe[0] = yThe0;
  zThe[0] = zThe0; 
  
  int t = 1/frequency;  //Finds time step process
  
  for(int i = 0; i < 1000; i++)
  {
    curtime[i] = curtime[i-1] + t;
  }  
  
  for(int i = 1; i < (sizeof(xOme)-2); i++)
  { //Calculates angle for absolute acceleration
    xThe[i+1] = xThe[i] + 0.5*t*(xOme[i] + xOme[i+1]);
    yThe[i+1] = yThe[i] + 0.5*t*(yOme[i] + yOme[i+1]);
    zThe[i+1] = zThe[i] + 0.5*t*(zOme[i] + zOme[i+1]);
  }   
  
  for(int i = 1; i < (sizeof(xAcc)-1); i++)
  { //Calculates absolute acceleration at each time point
    xAcca[i] = cos(yThe[i]*PI/180)*cos(zThe[i]*PI/180)*xAcc[i] + cos(yThe[i]*PI/180)*sin(zThe[i]*PI/180)*yAcc[i] + (-sin(yThe[i]*PI/180)*zAcc[i]);
    yAcca[i] = ((-cos(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + (cos(xThe[i]*PI/180)*cos(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + sin(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    zAcca[i] = (sin(xThe[i]*PI/180)*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + ((-sin(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + cos(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    zAcca[i] = zAcca[i] + 9.81;
  }  
  
  for(int i = 1; i < (sizeof(xAcca)-2); i++)
  { //Calculates velocity at each time point
    xVel[i+1] = xVel[i] + 0.5*t*(xAcca[i] + xAcca[i+1]);
    yVel[i+1] = yVel[i] + 0.5*t*(yAcca[i] + yAcca[i+1]);
    zVel[i+1] = zVel[i] + 0.5*t*(zAcca[i] + zAcca[i+1]);
  }
  
  for(int i = 1; i < (sizeof(xVel)-2); i++)
  { //calculates distance, angle at each time point
    xDist[i+1] = xDist[i] + 0.5*t*(xVel[i] + xVel[i+1]);
    yDist[i+1] = yDist[i] + 0.5*t*(yVel[i] + yVel[i+1]);
    zDist[i+1] = zDist[i] + 0.5*t*(zVel[i] + zVel[i+1]);
  }
  
  apogee = 0;
  for(int i = 0; i < (sizeof(zDist) - 1); i++)
  {
    if(apogee < zDist[i])
    {
      apogee = zDist[i];
    }
  }
  
  finCoords[0] = (xDist[sizeof(xDist)-1])*3.28084;
  finCoords[1] = (yDist[sizeof(yDist)-1])*3.28084;
  finCoords[2] = (zDist[sizeof(zDist)-1])*3.28084;
  //xOff = -250; //x and y offset of launchpad from gridded image origin. Enter the x and y components of the distance offset.
  //yOff = 250;
  
  xLoc = floor(((finCoords[1])+xOff)/250); //Finds the upper right-hand node of the cell that contains the location of the rocket
  yLoc = ceil((finCoords[1]+yOff)/250);
  
  cellID = 211 + xLoc - 20*yLoc; //Traces from origin to determine the cell number. Can be calculated.
  Serial.println(cellID);
  return cellID;
}
