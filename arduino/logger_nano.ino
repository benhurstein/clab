

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
 ** CS - pin 10

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe
 alterado por benhur e Caio em 31 maio 2015

 This example code is in the public domain.

 */

//#include <SPI.h>
#include <SD.h>

char fileName[] = "nanolog.txt";
const int chipSelect = 10;  // podem ocorrer problemas com esta porta

int Pin_Analog[8]   = { A0, A1, A2, A3, A4, A5, A6, A7 };
int Pin_Digital[8]  = { 2, 3, 4, 5, 6, 7, 8, 9 };

File dataFile;
long t0, t1;


union {
  byte buf[];
  struct {
    int32_t  timeStamp;
    int16_t  analog[8];
    int8_t  digital[8];
  } s;
} u;

int nbuf = 0;
int treg = 0;   // tamanho do registro que sera armazenado no arquivo do SD

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
  SD.remove(fileName);

  // set de pinmode
  for (int i = 0; i < 8; i++) {
    pinMode(Pin_Analog[i], INPUT);
    digitalWrite(Pin_Analog[i], LOW);
    pinMode(Pin_Digital[i], INPUT);
    digitalWrite(Pin_Digital[i], LOW);
  }
  
  treg = sizeof(u);  // seta o tamanho em bytes do registro
  
  t0 = millis();
}

void loop()
{
  for (int i = 0; i < 8; i++) {
    u.s.analog[i]   = analogRead(Pin_Analog[i]);
    u.s.digital[i]  = digitalRead(Pin_Digital[i]);
  }
  t1 = micros();
  u.s.timeStamp = t1;

  nbuf++;

  while (!dataFile)
    dataFile = SD.open(fileName, FILE_WRITE);

  int j = dataFile.write(u.buf, treg);
  if (nbuf % 1000 == 0) {
    Serial.println(dataFile);

    dataFile.close();
    print_buf(j, treg);
    Serial.println((1000.) / ((t1 - t0) / 1000000.));
    t0 = t1;
  }

}

void print_buf(int j, int z) {
  Serial.println(fileName);
  Serial.println(u.s.timeStamp);
  Serial.print(j);
  Serial.print(" bytes de ");
  Serial.println(z);
  for (int i = 0; i < 8; i++) {
    Serial.print(i);
    Serial.print(" / ");
    Serial.print(u.s.analog[i]);
    Serial.print(" / ");
    Serial.print(u.s.digital[i]);
    Serial.print("\t");
  }
}
