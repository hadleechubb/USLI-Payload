#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

#define SPI_SPEED SD_SCK_MHZ[50]12
#define GREEN_LED 4
#define BLUE_LED 5

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
int loops = 0;
char line[30];
bool transmit = false;
int timestamptracker = 0;
int TimeStamps[1000]; // check if needs to be higher
float frequency;

// Function Variables:
float xAcc[1001]; //Initializing acceleration arrays
float yAcc[1001];
float zAcc[1001];

float xAcca[1001]; //Initializing absolute acceleration arrays
float yAcca[1001];
float zAcca[1001];

float acca1[3][3];
float acca2[3][3];
float acca3[3][3];
float acc[3];
float acca[3][3];

float xOme[1001];
float yOme[1001];
float zOme[1001];

float xVel[1001]; //Initializing velocity arrays
float yVel[1001];
float zVel[1001];

float xDist[1001]; //Initializing position arrays
float yDist[1001];
float zDist[1001];

float xThe[1001]; //Initializing angle arrays
float yThe[1001];
float zThe[1001];

float finCoords[3];
float xLoc;
float yLoc;
int xOff;
int yOff;
byte cellID;
int fileposition;
float t;

float xyforces;
int fcount;
int begintime;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  
  Serial.print("Initializing SD card...");
  sd.begin(chipSelect, SPI_HALF_SPEED);
  Serial.println("initialization done.");

  myFile.open("CompetitonLaunch.csv", O_RDWR | O_CREAT | O_AT_END);
  myFile.println("Acceleration/Gyroscope");
  myFile.close();

    // set up LEDs
  pinMode(GREEN_LED,OUTPUT);
  digitalWrite(GREEN_LED, LOW);

  pinMode(BLUE_LED,OUTPUT);
  digitalWrite(BLUE_LED, LOW);
  
  Serial.print("Initializing IMU...");
  detectError = bno.begin();
  delay(100);
  while(detectError == 0)
  {
    digitalWrite(GREEN_LED,LOW);
    detectError = bno.begin();
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No BNO055 detected");
    myFile.open("CompetitonLaunch.csv", O_RDWR | O_AT_END);
    myFile.println("No BNO055 detected");
    myFile.close();
    delay(100);
    Serial.print(".");
  }
  digitalWrite(GREEN_LED,HIGH);
  Serial.println(" IMU initialization complete.");
  landed = false;
  begintime = millis();
}

void loop()
{
  if(landed == false)
  {
    Serial.println("Taking Data");
    myFile.open("CompetitonLaunch.csv", O_RDWR | O_AT_END);
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
      xyforces = sqrt(sq(accldata.x())+sq(accldata.y()));
      if(xyforces > 8.5 && xyforces < 10.5)
        fcount++;
      else
        fcount = 0;
    }
    myFile.println(millis());
    myFile.close();
    loops++;
    Serial.println(fcount);
    if(fcount > 15000)
      landed = true;
    if(millis()-begintime > 5400000)
      landed = true;
  }

  if(landed == true)
  {
    if(transmit == false)
    {
      Serial.print("Calculating...");
      myFile.open("CompetitonLaunch.csv", O_READ);
      myFile.fgets(line,sizeof(line));
      while((line[0] == 'A') || (line[0] == 'N'))
      {
        myFile.fgets(line,sizeof(line));
      }
      fileposition = myFile.position()-(sizeof(atoi(line))+2);
      myFile.close();
      finalLoc(loops, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);         
      xLoc = floor(((finCoords[0])+xOff)/250); //Finds the upper right-hand node of the cell that contains the location of the rocket
      yLoc = ceil((finCoords[1]+yOff)/250);
      
      cellID = 211 + xLoc - 20*yLoc; //Traces from origin to determine the cell number. Can be calculated.
      Serial.print("cellID: ");
      Serial.println(cellID); 
      myFile.open("CompetitonLaunch.csv",O_READ);
      myFile.println("finCoords:");
      myFile.println(finCoords[0]);
      myFile.println(finCoords[1]);
      myFile.println(finCoords[2]);
      myFile.println("cell ID:");
      myFile.println(cellID);
      myFile.close();
      transmit = true;
      digitalWrite(BLUE_LED, HIGH);
    }
  }
}



// location calculation function
void finalLoc(int loops, float xVel0, float yVel0, float zVel0, float xDist0, float yDist0, float zDist0, float xThe0, float yThe0, float zThe0, float xAcca0, float yAcca0, float zAcca0, float xOme0, float yOme0, float zOme0)
{
  myFile.open("CompetitonLaunch.csv", O_RDWR); //add position
  myFile.seek(fileposition);
  myFile.fgets(line,sizeof(line));
  while((line[0] == 'A') || (line[0] == 'N'))
  {
    myFile.fgets(line,sizeof(line));
  }
  TimeStamps[timestamptracker] = atoi(line);
  timestamptracker++;
//  Serial.println(TimeStamps[0]);
  for(int i = 1; i < 1001; i++)
  {
    // Accl X
    myFile.fgets(line, sizeof(line));
    xAcc[i] = atof(line);
//    Serial.print("xAcc: ");
//    Serial.println(xAcc[i]);
    // Accl Y
    myFile.fgets(line, sizeof(line));
    yAcc[i] = atof(line)-1.56;
//    Serial.print("yAcc: ");
//    Serial.println(yAcc[i]);
    // Accl Z
    myFile.fgets(line, sizeof(line));
    zAcc[i] = -atof(line)-0.4;
//    Serial.print("zAcc: ");
//    Serial.println(zAcc[i]);
    // Gyro X
    myFile.fgets(line, sizeof(line));
    xOme[i] = -atof(line);
//    Serial.print("xOme: ");
//    Serial.println(xOme[i]);
    // Gyro Y
    myFile.fgets(line, sizeof(line));
    yOme[i] = -atof(line);
//    Serial.print("yOme: ");
//    Serial.println(yOme[i]);
    // Gyro Z
    myFile.fgets(line, sizeof(line));
    zOme[i] = atof(line);
//    Serial.print("zOme: ");
//    Serial.println(zOme[i]);
  }
  // get ending time stamp
  myFile.fgets(line,sizeof(line));
  TimeStamps[timestamptracker] = atoi(line);
  timestamptracker++;
  fileposition = myFile.position();
  myFile.close(); 
  loops--;

  frequency = (TimeStamps[timestamptracker-1]-TimeStamps[timestamptracker-2]);
  frequency = frequency/1000;
  frequency = 1000/frequency;
  
  xVel[0] = xVel0;
  yVel[0] = yVel0;
  zVel[0] = zVel0;

  xDist[0] = xDist0;
  yDist[0] = yDist0;
  zDist[0] = zDist0; 

  xThe[0] = xThe0;
  yThe[0] = yThe0;
  zThe[0] = zThe0; 

  xOme[0] = xOme0;
  yOme[0] = yOme0;
  zOme[0] = zOme0;

  xAcca[0] = xAcca0;
  yAcca[0] = yAcca0;
  zAcca[0] = zAcca0;

  t = 1.00/frequency;  //Finds time step process
  
  for(int i = 1; i < 1001; i++) //0th element of array already set as initial
  { //Calculates angle for absolute acceleration
    xThe[i] = xThe[i-1] + 0.5*t*(xOme[i-1] + xOme[i]);
//    Serial.print("xThe: ");
//    Serial.println(xThe[i]);
    yThe[i] = yThe[i-1] + 0.5*t*(yOme[i-1] + yOme[i]);
//    Serial.print("yThe: ");
//    Serial.println(yThe[i]);
    zThe[i] = zThe[i-1] + 0.5*t*(zOme[i-1] + zOme[i]);
//    Serial.print("zThe: ");
//    Serial.println(zThe[i]);
  }   
//  Serial.print("xAcca[0]:");
//  Serial.println(xAcca[0]);


  for(int i = 1; i < 1001; i++)
  { //Calculates absolute acceleration at each time point
    xAcca[i] = cos(yThe[i]*PI/180)*cos(zThe[i]*PI/180)*xAcc[i] + cos(yThe[i]*PI/180)*sin(zThe[i]*PI/180)*yAcc[i] + (-sin(yThe[i]*PI/180)*zAcc[i]);
//    Serial.print("Absolute X Acc:");
//    Serial.println(xAcca[i]);
    yAcca[i] = ((-cos(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + (cos(xThe[i]*PI/180)*cos(zThe[i]*PI/180)+sin(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + sin(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    // technically don't need to save or calculate z
//    Serial.print("Absolute Y Acc:");
//    Serial.println(yAcca[i]);
    zAcca[i] = (sin(xThe[i]*PI/180)*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*cos(zThe[i]*PI/180))*xAcc[i] + ((-sin(xThe[i]*PI/180))*sin(zThe[i]*PI/180)+cos(xThe[i]*PI/180)*sin(yThe[i]*PI/180)*sin(zThe[i]*PI/180))*yAcc[i] + cos(xThe[i]*PI/180)*cos(yThe[i]*PI/180)*zAcc[i];
    zAcca[i] = zAcca[i];// + 9.81;
//    Serial.print("Absolute Z Acc:");
//    Serial.println(zAcca[i]);
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
    finalLoc(loops, xVel[1000], yVel[1000], zVel[1000], xDist[1000], yDist[1000], zDist[1000], xThe[1000], yThe[1000], zThe[1000], xAcca[1000], yAcca[1000], zAcca[1000], xOme[1000], yOme[1000], zOme[1000]);
  }
  else
  {
    finCoords[0] = (xDist[1000])*3.28084;
    Serial.println(finCoords[0]);
    finCoords[1] = (yDist[1000])*3.28084;
    Serial.println(finCoords[1]);
    finCoords[2] = (zDist[1000])*3.28084;
    Serial.println(finCoords[2]);
  }
}
