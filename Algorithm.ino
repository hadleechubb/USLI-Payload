#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define SPI_SPEED SD_SCK_MHZ[50]12

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
bool detectError;
bool landed;
int loops = 338;
char line[10];
bool transmit = false;
int timestamptracker = 0;
int TimeStamps[1000]; // check if needs to be higher
float frequency;
byte headercount = 3; //should start at 1 for "Acceleration/Gyroscope" header, increment by 1 for every "No BNO055 detected" written

// Function Variables:
int xAcc[1001]; //Initializing acceleration arrays
int yAcc[1001];
int zAcc[1001];

int xAcca[1001]; //Initializing absolute acceleration arrays
int yAcca[1001];
int zAcca[1001];

float acca1[3][3];
float acca2[3][3];
float acca3[3][3];
int acc[3];
float acca[3][3];

int xOme[1001];
int yOme[1001];
int zOme[1001];

int xVel[1001]; //Initializing velocity arrays
int yVel[1001];
int zVel[1001];

int xDist[1001]; //Initializing position arrays
int yDist[1001];
int zDist[1001];

int xThe[1001]; //Initializing angle arrays
int yThe[1001];
int zThe[1001];

int finCoords[3];
int xLoc;
int yLoc;
int xOff;
int yOff;
byte cellID;
int fileposition;

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
  //set xOff and yOff
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
      myFile.open("AlgorithmTest.csv", O_RDWR);
      for(int j = 0; j < headercount; j++)
      {
        myFile.fgets(line,sizeof(line));
      }
      fileposition = myFile.position();
      Serial.print("position: ");
      Serial.println(fileposition);
      myFile.close();
      finalLoc(loops, 0, 0, 0, 0, 0, 0, 0, 0, 0);         
      xLoc = floor(((finCoords[0])+xOff)/250); //Finds the upper right-hand node of the cell that contains the location of the rocket
      yLoc = ceil((finCoords[1]+yOff)/250);
      
      cellID = 211 + xLoc - 20*yLoc; //Traces from origin to determine the cell number. Can be calculated.
      Serial.print("cellID: ");
      Serial.println(cellID); 
      transmit = true;    
    }
  }
}



// location calculation function
void finalLoc(int loops, int xVel0, int yVel0, int zVel0, int xDist0, int yDist0, int zDist0, int xThe0, int yThe0, int zThe0)
{
  //@kendra calculate position from loops
  myFile.open("AlgorithmTest.csv", O_RDWR); //add position
  myFile.seek(fileposition);
  myFile.fgets(line,sizeof(line));
  TimeStamps[timestamptracker] = atoi(line);
  timestamptracker++;
  for(int i = 1; i<1001; i++)
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
    xOme[i] = -atoi(line);
  }
  // get ending time stamp
  myFile.fgets(line,sizeof(line));
  TimeStamps[timestamptracker] = atoi(line);
  timestamptracker++;

  fileposition = myFile.position();
  Serial.print("position: ");
  Serial.println(fileposition);
  myFile.close(); 
  loops--;

  frequency = 6000/(TimeStamps[timestamptracker]-TimeStamps[timestamptracker-1]);  //@kendra check this
  Serial.print("frequency: ");
  Serial.println(frequency);

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
  
  for(int i = 1; i < 1001; i++) //0th element of array already set as initial
  { //Calculates angle for absolute acceleration
    xThe[i] = xThe[i-1] + 0.5*t*(xOme[i-1] + xOme[i]);
    yThe[i] = yThe[i-1] + 0.5*t*(yOme[i-1] + yOme[i]);
    zThe[i] = zThe[i-1] + 0.5*t*(zOme[i-1] + zOme[i]);
  }   
  
  for(int i = 0; i < 1001; i++)
  { //Calculates absolute acceleration at each time point
    xAcca[i] = cos(yThe[i])*cos(zThe[i]*PI/180)*xAcc[i] + cos(yThe[i]*PI/180)*sin(zThe[i]*PI/180)*yAcc[i] + (-sin(yThe[i]*PI/180)*zAcc[i]);
    yAcca[i] = ((-cos(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + (cos(xThe[i]*PI/180)*cos(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + sin(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    // technically don't need to save or calculate z
    zAcca[i] = (sin(xThe[i]*PI/180)*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + ((-sin(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + cos(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    zAcca[i] = zAcca[i] + 9.81;
  }  
  
  for(int i = 1; i < 1001; i++) //0th element of array already set
  { //Calculates velocity at each time point
    xVel[i] = xVel[i-1] + 0.5*t*(xAcca[i-1] + xAcca[i]);
    yVel[i] = yVel[i-1] + 0.5*t*(yAcca[i-1] + yAcca[i]);
    zVel[i] = zVel[i-1] + 0.5*t*(zAcca[i-1] + zAcca[i]);
  }
  
  for(int i = 1; i < 1001; i++) //0th element of array already set
  { //calculates distance, angle at each time point
    xDist[i] = xDist[i-1] + 0.5*t*(xVel[i-1] + xVel[i]);
    yDist[i] = yDist[i-1] + 0.5*t*(yVel[i-1] + yVel[i]);
    zDist[i] = zDist[i-1] + 0.5*t*(zVel[i-1] + zVel[i]);
  }
  if(loops!= 0)
  {
    finalLoc(loops, xVel[999], yVel[999], zVel[999], xDist[999], yDist[999], zDist[999], xThe[999], yThe[999], zThe[999]);
  }
  else
  {
    finCoords[0] = (xDist[999])*3.28084;
    finCoords[1] = (yDist[999])*3.28084;
    // finCoords[2] = (zDist[999])*3.28084;
  }
}
