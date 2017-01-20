

/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"


const int chipSelect = 4;

char fileName[] = "GI000000";
File dataFile;
long t0, t1;

union {
  byte buf[];
  struct {
    int32_t timeStamp;
    int16_t v1, v2, v3;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
  } s;
} u;

int nbuf = 0;

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high


void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  int i = 0;
  while (SD.exists(fileName)) {
    Serial.print("Ja existe ");
    Serial.println(fileName);
    i++;
    itoa(i, fileName+2, 10);
  }
  
  while (!dataFile)
    dataFile = SD.open(fileName, FILE_WRITE);
  Serial.print("Gravando em ");
  Serial.println(fileName);

  Wire.begin();
  accelgyro.initialize();


  t0 = millis();
}

void loop()
{
  accelgyro.getMotion6(&u.s.ax, &u.s.ay, &u.s.az, 
                       &u.s.gx, &u.s.gy, &u.s.gz);
  u.s.v1 = analogRead(0);
  u.s.v1 = analogRead(1);
  u.s.v1 = analogRead(2);
  t1 = micros();
  u.s.timeStamp = t1;

  nbuf++;
  
  //while (!dataFile)
  //  dataFile = SD.open(fileName, FILE_WRITE);

  dataFile.write(u.buf, sizeof(u.buf));
  if (nbuf % 1000 == 0) {
    dataFile.flush();
    Serial.print(u.s.timeStamp);
    Serial.print(" ");
    Serial.print(u.s.ax);
    Serial.print(" ");
    Serial.println((1000.) / ((t1 - t0)/1000000.));
    t0 = t1;
  }

}


