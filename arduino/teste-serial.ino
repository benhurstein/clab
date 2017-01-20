long now;

#include <HardwareSerial_private.h>

class NHardwareSerial : public HardwareSerial
{
  public:
    void disabletx(void);
    void enabletx(void);
    void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }
    void begin(unsigned long, uint8_t);
};

void NHardwareSerial::disabletx(void)
{
  flush();
  cbi(*_ucsrb, TXEN0); 
  pinMode(1, INPUT);
  //digitalWrite(1, HIGH);
}
void NHardwareSerial::enabletx(void)
{
  sbi(*_ucsrb, TXEN0); 
}
void NHardwareSerial::begin(unsigned long baud, byte config)
{
  // Try u2x mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  *_ucsra = 1 << U2X0;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
  {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  _written = false;

  //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
  config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
  *_ucsrc = config;

  sbi(*_ucsrb, RXEN0);
  cbi(*_ucsrb, TXEN0);
  sbi(*_ucsrb, RXCIE0);
  cbi(*_ucsrb, UDRIE0);
}

#define NSerial (*(NHardwareSerial*)&Serial)

#include <OneWire.h>

#define PIN_TEMP 4 // temperaturas
OneWire sensors(PIN_TEMP);

// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  float celsius; // temperatura (passa-baixa da temperatura lida)
  byte addr[8]; // endereco do sensor
  long delta;
} temp;

#define NTEMPS 10
temp temps[NTEMPS];
byte ntemps = 0;
bool temtemps = false;

void find_temps(void)
{
  int t;
  for (t=0; t<NTEMPS; t++) {
    //Serial.print("Searching "); Serial.print(t); Serial.println(" ");
    if (!sensors.search(temps[t].addr)) {
      //Serial.println("END");
      break;
    }
    //printAddr(temps[t].addr);
    if (OneWire::crc8(temps[t].addr, 7) != temps[t].addr[7]) {
      //Serial.println("CRC is not valid!");
      t--;
      continue;
    }
    // the first ROM byte indicates which chip
    if (temps[t].addr[0] != 0x28) {
        //Serial.println("Device is not a DS18B20.");
        t--;
        continue;
    }
    temps[t].celsius = NAN;
  }
  ntemps = t;
}

void start_conversion(byte t)
{
    sensors.reset();
    //sensors.select(temps[t].addr);
    sensors.write(0xcc);
    sensors.write(0x44);//, 1);        // start conversion, with parasite power on at the end
}

float low_pass(float old_value, float new_value)
{
  if (isnan(old_value)) return new_value;
  if (isnan(new_value)) return old_value;
  return old_value * 0.4 + 0.6 * new_value;

}

float read_temp(byte t)
{
  byte i;
  byte data[9];
  byte crc;
  
  sensors.reset();
  sensors.select(temps[t].addr);    
  sensors.write(0xBE);         // Read Scratchpad

  /*
  Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");
  */
  for (i = 0; i < 2/*9*/; i++) {           // we need 9 bytes
    data[i] = sensors.read();
  }
  /*
  crc = OneWire::crc8(data, 8);
  if (crc != data[8]) {
    return NAN;
  }
  */
  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  /*
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  //// default is 12 bit resolution, 750 ms conversion time
  */
  return (float)raw / 16.0;
}

void read_temps(void)
{
  static byte t;
  static byte status = 0;
  static long timeStamp;

  if (status == 0) {
    //for (t=0; t<ntemps; t++) {
      start_conversion(t);
    //}
    status = 1;
    t = 0;
    timeStamp = now;
  } else {
    if (!temtemps && (now - timeStamp) > 750) {
      //for (t=0; t<ntemps; t++) {
        float celsius = read_temp(t);
        temps[t].celsius = celsius;//low_pass(temps[t].celsius, celsius);
        temps[t].delta = millis() - now;
      //}
      t++;
      if (t<ntemps) return;
      status = 0;
      timeStamp = now;
      temtemps = true;
    }
  }
}



void setup() {
  // put your setup code here, to run once:
  NSerial.begin(115200);
  find_temps();
}

void loop() {
  byte b;

  now = millis();

  read_temps();

  if (NSerial.available()) {
    //delay(1000);
    while (NSerial.available()) {
      b = NSerial.read();
      if (b == '1') {
        long nnn = millis();
        NSerial.enabletx();
        NSerial.print("1 ");
        if (temtemps){
          temtemps=false;
        NSerial.print(now);
        for (byte i =0; i<ntemps; i++) {
          NSerial.print(" ");
          NSerial.print(temps[i].celsius);
          NSerial.print(" ");
          NSerial.print(temps[i].delta);
        }
        NSerial.print(" ");
        NSerial.flush();
        NSerial.print(millis() - nnn);
        }else{
          NSerial.print("X");
        }
        NSerial.println("");
        NSerial.disabletx();
      }
    }
    //NSerial.print('.');
    //delay(1000);
  }
}
