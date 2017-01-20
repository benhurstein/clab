#include <OneWire.h>
#include <TM1638.h>

OneWire ds/*temp*/(4);
TM1638 disp(5, 6, 7, true, 7);

#define T_IN_HOT   0
#define T_OUT_HOT  1
#define T_IN_COLD  2
#define T_OUT_COLD 3
float temps[4];
int nt = 0;

// cabo até piscina
// no arduino: l/bl: -+ tx
//             v/bv: -+ rx
//             a/ba: +12V
//             m/bm: 0V

long timeStamp;
long timeOut = 0;
bool motorLigado = false;

void setup() {
  Serial.begin(9600);

  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  disp.setDisplayToString("Start");
  delay(1000);
}

void displayTwoTemps(float t1, float t2)
{
  unsigned long a, b;
  unsigned long n;
  a = t1 * 100;
  b = t2 * 100;
  a = a % 10000;
  b = b % 10000;
  n = a*10000 + b;
  disp.setDisplayToDecNumber(n, 68, true);
/*
  Serial.print(t1);  Serial.print(" ");
  Serial.print(t2);  Serial.print(" ");
  Serial.print(a);  Serial.print(" ");
  Serial.print(b);  Serial.print(" ");
  Serial.print(n);  Serial.println(" ");
*/
}

float hi_temp = -100;

void loop() {
  byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius, fahrenheit;

//temps[0] = 52; if(1){delay(1250);}else{

  if ( !ds.search(addr)) {
    if (nt < 4) {
      Serial.print("Need 4 sensors, found ");
      Serial.println(nt);
      disp.setDisplayToDecNumber(nt, 255, true);
    }
    /*
    Serial.println("No more addresses.");
    Serial.println();
    */
    ds.reset_search();
    nt = 0;
    hi_temp = -100;
    delay(250);
    return;
  }

  /*
  Serial.print("ROM =");
  for( i = 0; i < 8; i++) {
    Serial.write(' ');
    Serial.print(addr[i], HEX);
  }
  */
  
  if (OneWire::crc8(addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(1000);     // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  /*
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  */
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  /*
  Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
  */

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  /*
  fahrenheit = celsius * 1.8 + 32.0;
  Serial.print("  Temperature = ");
  Serial.print(celsius);
  Serial.print(" Celsius, ");
  Serial.print(fahrenheit);
  Serial.println(" Fahrenheit");
  */
  if (celsius == 85 || raw == -1) return;
  if (temps[nt] == 0) temps[nt] = celsius;
  else temps[nt] = temps[nt] * 0.6 + 0.4 * celsius;
  if (temps[nt] > hi_temp) hi_temp = temps[nt];
  Serial.print(nt); Serial.print(" "); Serial.print(celsius); Serial.print(" "); Serial.print(hi_temp);
//}if(nt==4)nt=0;
  nt++;
  
  if((nt) == 2) {
    displayTwoTemps(temps[nt-2], temps[nt-1]);
    disp.setLED(TM1638_COLOR_RED, 0);
    disp.setLED(TM1638_COLOR_NONE, 1);
  }
  if((nt) == 4) {
    displayTwoTemps(temps[nt-2], temps[nt-1]);
    disp.setLED(TM1638_COLOR_NONE, 0);
    disp.setLED(TM1638_COLOR_GREEN, 1);
  }

  timeStamp = millis();
//  if (temps[T_IN_HOT] > 48) {
  if (hi_temp > 40) {
    motorLigado = true;
  }
//  if (motorLigado && temps[T_IN_HOT] > 43) {
  if (motorLigado && hi_temp > 35) {
    timeOut = timeStamp + 15 * 60L * 1000;
  }
  if (motorLigado && timeOut <= timeStamp) {
    motorLigado = false;
  }
  Serial.print(" "); Serial.println(motorLigado);

  if (motorLigado) {
    digitalWrite(8, LOW);
  } else {
    digitalWrite(8, HIGH);
  }

  disp.setLED((timeOut-timeStamp)>10*60L*1000, 2);
  disp.setLED((timeOut-timeStamp)>8*60L*1000, 3);
  disp.setLED((timeOut-timeStamp)>6*60L*1000, 4);
  disp.setLED((timeOut-timeStamp)>4*60L*1000, 5);
  disp.setLED((timeOut-timeStamp)>2*60L*1000, 6);
  disp.setLED((timeOut-timeStamp)>0*60L*1000, 7);

}
