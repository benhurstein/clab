#define NODE_ID 'A'  // this is the wine cellar node A

//#include <DHT.h>
#include <Wire.h>
//#include <AM2320.h>
#include <OneWire.h>
#define AM2320_address (0xB8 >> 1)
#define PIN_TEMP 4

// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  int16_t raw;   // dado cru da temperatura
  float celsius; // temperatura (passa-baixa da temperatura lida)
  byte addr[8];  // endereco do sensor
} temp;

#define NTEMPS 8
  temp temps[NTEMPS];
  byte ntemps = 0;
  bool hasNewTemps = false;

  uint32_t now;
  uint32_t htTimestamp;

  float hthum;
  float httmp;
  bool hasNewHT = false;

  byte ht_1w_pin = 7;
  float ht_1w_hum;
  float ht_1w_temp;
  bool hasNewHT1w = false;

OneWire ds(PIN_TEMP);

void find_temps(void)
{
  int t;
  Serial.print("search");
  for (t = 0; t < NTEMPS; t++) {
    if (!ds.search(temps[t].addr)) {
      Serial.println(" end search");
      break;
    }
    if (OneWire::crc8(temps[t].addr, 7) != temps[t].addr[7]) {
      // CRC is not valid!
      t--;
      continue;
    }
    // the first ROM byte indicates which chip
    if (temps[t].addr[0] != 0x28) {
      // Device is not a DS18B20
      t--;
      continue;
    }
    temps[t].celsius = NAN;
    byte i; for(i=0;i<8;i++) {Serial.print(temps[t].addr[i], HEX); Serial.print(" ");}Serial.println();
  }
  ntemps = t;
}

void start_conversion(void)
{
    ds.reset();
    //ds.select(temps[t].addr);
    ds.write(0xcc); // envia a todos os sensores
    ds.write(0x44); // pedido de início de conversão
}

float low_pass(float old_value, float new_value)
{
  if (isnan(old_value)) return new_value;
  if (isnan(new_value)) return old_value;
  if (new_value - old_value > 5 || new_value - old_value < -5) return old_value;
  return old_value * 0.4 + 0.6 * new_value;

}

int16_t read_temp(byte t)
{
  byte b0, b1;
  
  ds.reset();
  ds.select(temps[t].addr);    
  ds.write(0xBE);         // Read Scratchpad

  b0 = ds.read();
  b1 = ds.read();

  return b1 << 8 | b0;
}

float raw_to_celsius(int16_t raw)
{
  float c = raw / 16.0f;
  if (c == 85 || c == 0) return NAN;
  if (c < -55 || c > 125) return NAN;
  return c;  
}

void read_temps(void)
{
  static byte t;
  static enum { stopped, converting, reading } status = stopped;
  static uint32_t timeStamp;

  switch (status) {
    case stopped: 
      if ((now - timeStamp) >= 1000) {
      start_conversion();
      timeStamp = now;
      status = converting;
      }
      break;
    case converting:
      if ((now - timeStamp) >= 750) {
        t = 0;
        status = reading;
      }
      break;
    case reading:
      if (t < ntemps) {
        temps[t].raw = read_temp(t);
        float celsius = raw_to_celsius(temps[t].raw);
        temps[t].celsius = celsius;//low_pass(temps[t].celsius, celsius);
        t++;
      }
      if (t >= ntemps) {
        hasNewTemps = true;
        status = stopped;
        timeStamp = now;
      }
  }
}

unsigned int crc16(byte *ptr, byte length)
{
      unsigned int crc = 0xFFFF;
      uint8_t s = 0x00;

      while(length--) {
        crc ^= *ptr++;
        for(s = 0; s < 8; s++) {
          if((crc & 0x01) != 0) {
            crc >>= 1;
            crc ^= 0xA001;
          } else crc >>= 1;
        }
      }
      return crc;
}

void read_ht(void)
{
  static byte t;
  static enum { stopped, waking, waiting, reading } status = stopped;
  static uint32_t timeStamp;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 60000) {
        Wire.beginTransmission(AM2320_address); // ask it to wake up
        status = waking;
        timeStamp = now;
      }
      break;
    case waking:
      if (now - timeStamp >= 1) { // must wait at least 0.8 ms
        Wire.endTransmission(); // should be awaken
        Wire.beginTransmission(AM2320_address);
        Wire.write(3); // read
        Wire.write(0); // from address 0
        Wire.write(4); // 4 bytes
        if (Wire.endTransmission(1) == 0) {
          status = waiting;
          timeStamp = now;
        } else {
          // something went wrong -- restart
          status = stopped;
          timeStamp = now - (2500 - 1); // start again in 1 ms
        }
      }
      break;
    case waiting:
      if (now - timeStamp > 2) { // must wait at least 1.5 ms
        Wire.requestFrom(AM2320_address, 0x08);
        t = 0;
        status = reading;
        timeStamp = now;
      }
      break;
    case reading:
      static byte buf[8];
      buf[t] = Wire.read();
      if (t < 7) {
        t++;
      } else {
        uint16_t crc = buf[7] << 8 | buf[6];
        if (crc == crc16(buf, 6)) {
          hthum = (buf[2] << 8 | buf[3]) / 10.0;
          httmp = ((buf[4] & 0x7f) << 8 | buf[5]) / 10.0;
          if (buf[4] & 0x80) httmp = -httmp;
          hasNewHT = true;
        }
        status = stopped;
        timeStamp = now;
      }
      break;
  }
}

void read_ht_1w(void)
{
  static enum { stopped, waking, reading } status = stopped;
  static uint32_t timeStamp;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 60000) {
        // wake the sensor up
        pinMode(ht_1w_pin, OUTPUT);
        digitalWrite(ht_1w_pin, LOW);
        status = waking;
        timeStamp = now;
      }
      break;
    case waking:
      if (now - timeStamp > 2) {
        // it should be awaken, let's read
        status = reading;
        timeStamp = now;
      }
      break;
    case reading:
      byte buf[5];
      // max number of times to read the pin value
      // so that in case of comm error we don't get stuck
      // value must be changed for faster processor
      uint16_t timeout = 1000;

      // time critical -- cannot be interrupted
      noInterrupts();
      // let the sensor be in control
      pinMode(ht_1w_pin, INPUT_PULLUP);
      while (timeout && digitalRead(ht_1w_pin) == LOW) timeout--;
      // the sensor should put the pin on low and then on high, for ~80us each
      while (timeout && digitalRead(ht_1w_pin) == HIGH) timeout--;
      while (timeout && digitalRead(ht_1w_pin) == LOW) timeout--;
      // now it should put it on low to start the first bit
      while (timeout && digitalRead(ht_1w_pin) == HIGH) timeout--;

      // read 5 bytes
      for (int i=0; timeout && i<5; i++) {
        byte b; // accumulate the bits here before putting in buf
        // for each byte, we need 8 bits
        for (byte bit=0; bit<8; bit++) {
          byte count_low = 0, count_high = 0;
          b <<= 1;
          while (timeout && digitalRead(ht_1w_pin) == LOW) {
            count_low += 3;
            timeout--;
          }
          while (timeout && digitalRead(ht_1w_pin) == HIGH) {
            count_high += 4;
            timeout--;
          }
          if (count_high > count_low) { // bit is 1 if more time on high than low
            b |= 1;
          } // else, bit on b is already 0
        }
        buf[i] = b;
      }
      interrupts();
      // end of time-critical section

      if (buf[4] == ((buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF)) {
        ht_1w_hum = ((buf[0] << 8) | buf[1]) / 10.0;
        ht_1w_temp = (((buf[2] & 0x7f) << 8) | buf[3]) / 10.0;
        if (buf[2] & 0x80) ht_1w_temp = -ht_1w_temp;
        hasNewHT1w = true;
        //Serial.print(timeout);
      }
      status = stopped;
      timeStamp = now;
      break;
  }
}


// NSerial
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
// NSerial end

void verify_comm(void)
{
  byte b;
  if (NSerial.available()) {
    while (NSerial.available()) {
      b = NSerial.read();
      if (b == NODE_ID) {
        NSerial.enabletx();
        NSerial.print(NODE_ID);
        if (!hasNewTemps && !hasNewHT && !hasNewHT1w) {
           NSerial.print("X");
        } else {
          NSerial.print(" ");
          NSerial.print(now);
          if (hasNewHT) {
            hasNewHT = false;
            NSerial.print(" H ");
            NSerial.print(hthum);
            NSerial.print(" ");
            NSerial.print(httmp);
          } else if (hasNewHT1w) {
            hasNewHT1w = false;
            NSerial.print(" h ");
            NSerial.print(ht_1w_hum);
            NSerial.print(" ");
            NSerial.print(ht_1w_temp);
          } else if (hasNewTemps) {
            hasNewTemps = false;
            NSerial.print(" T");
            for (byte i = 0; i < ntemps; i++) {
              NSerial.print(" ");
              NSerial.print(temps[i].raw);
              NSerial.print(" ");
              NSerial.print(temps[i].celsius);
            }
          }
        }
        NSerial.print("\n");
        NSerial.disabletx();
      }
    }
  }
}



void setup() {
  //int i;
  //pinMode(A5, OUTPUT);
  //digitalWrite(A5, LOW);

  //dht[0].init(A4, AM2301);
  //dht[1].init(A4, AM2301);
  //pinMode(ht_1w_pin, INPUT_PULLUP);
  //pinMode(12, INPUT_PULLUP);
  Wire.begin();
  /*N*/Serial.begin(115200);

Serial.println("tst temps");
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

  //for (i = 0; i < N; i++)
  //  dht[i].begin();

  find_temps();
  //delay(2000);
}

void pisca1(byte b)
{
  if (b==0) {
    digitalWrite(13, HIGH);
    delay(20);
    digitalWrite(13, LOW);
    delay(200);
    
  } else
  while(b) {
    digitalWrite(13, HIGH);
    delay(200);
    digitalWrite(13, LOW);
    delay(200);
    b--;
  }
  delay(1000);
}
void pisca(float v)
{
  int vi = v*10;
  pisca1(vi/100%10);
  pisca1(vi/10%10);
  pisca1(vi%10);
}

void loop() {

static long ts = 0;
  now = millis();

  read_temps();
  //read_ht();
  //read_ht_1w();

  //verify_comm();
  
  if (hasNewTemps) {
    hasNewTemps = false;

    for (byte i = 0; i < ntemps; i++) {
      Serial.println(temps[i].celsius);
      pisca(temps[i].celsius);
    }
    find_temps();
    ts = now;

  }

  if (now - ts > 10000) {
    find_temps();
    ts = now;
  }
  
/*
  if (hasNewHT) {
    hasNewHT = false;
    Serial.print("HT");
    Serial.print(" ");
    Serial.print(hthum);
    Serial.print(" ");
    Serial.print(httmp);
    Serial.println();
  }

  if (hasNewHT1w) {
    hasNewHT1w = false;
    Serial.print("HT1W");
    Serial.print(" ");
    Serial.print(ht_1w_hum);
    Serial.print(" ");
    Serial.print(ht_1w_temp);
    Serial.println();
  }
  */
  return;
}
