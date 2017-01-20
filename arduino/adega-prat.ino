//#define NODE_ID 'A'  // this is the wine cellar node A
extern const uint8_t node_id = 0xfA;

#include "clab_msg.h"

//#include <DHT.h>
#include <Wire.h>
//#include <AM2320.h>
#include <OneWire.h>
#define AM2320_address (0xB8 >> 1)

#define PIN_TEMP 4
#define PIN_FAN_ON  11
#define PIN_FAN_SPEED 12

// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  byte addr[8];  // endereco do sensor
  int16_t raw;   // dado cru da temperatura
  bool valid;    // validade do dado em raw
} temp;

#define NTEMPS 8
  temp temps[NTEMPS] = {
    { { 28, 0xff, 0x98, 0x66, 0xa8, 0x15, 0x04, 0xf9 }, 0, false }, //
    { { 28, 0xff, 0x58, 0x60, 0xa8, 0x15, 0x04, 0xd2 }, 0, false }, //
    { { 28, 0xff, 0x4a, 0x1b, 0xa8, 0x15, 0x01, 0x2e }, 0, false }, //
    { { 28, 0xff, 0x49, 0x72, 0xa8, 0x15, 0x03, 0xdf }, 0, false }, //
    { { 28, 0xff, 0x39, 0x6e, 0xa8, 0x15, 0x04, 0x1b }, 0, false }, //
    { { 28, 0xff, 0x87, 0x77, 0xa8, 0x15, 0x03, 0xde }, 0, false }, //
  };
//  const uint8_t ntemps = sizeof(temps)/sizeof(temps[0]);
  uint8_t ntemps = 0;
  uint8_t validtemps = 0;
  bool hasNewTemps = false;

  uint32_t now;
  uint32_t htTimestamp;

  float hthum;
  int16_t hthum_raw;
  float httmp;
  int16_t httmp_raw;
  bool hasNewHT = false;

  byte ht_1w_pin = 6;
  float ht_1w_hum;
  int16_t ht_1w_hum_raw;
  float ht_1w_temp;
  int16_t ht_1w_temp_raw;
  bool hasNewHT1w = false;

OneWire ds(PIN_TEMP);

const byte PATa = 0xac;
const byte PATb = 0x65;

void temps_begin(void)
{
  ds.reset();
  ds.write(0xcc); // envia para todos os sensores
  ds.write(0x4e); // write scratchpad
  ds.write(PATa); // padrao de bits para conferir nas leituras
  ds.write(PATb); // outro padrao
  ds.write(0x7f); // configuracao - conversao de 12 bits
}



void find_temps(void)
{
  int t;
  for (t = 0; t < NTEMPS; t++) {
    if (!ds.search(temps[t].addr)) {
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
    //temps[t].celsius = NAN;
  }
  ntemps = t;
}


void start_conversion(void)
{
  ds.reset();
  ds.write(0xcc); // envia a todos os sensores
  ds.write(0x44); // pedido de início de conversão

  for (uint8_t i = 0; i < ntemps; i++) {
    temps[i].valid = false;
  }
  validtemps = 0;
}

/*
float low_pass(float old_value, float new_value)
{
  if (isnan(old_value)) return new_value;
  if (isnan(new_value)) return old_value;
  if (new_value - old_value > 5 || new_value - old_value < -5) return old_value;
  return old_value * 0.4 + 0.6 * new_value;

}
*/

void read_temp(byte t)
{
  byte b[9];
  
  ds.reset();
  ds.select(temps[t].addr);    
  ds.write(0xBE);         // Read Scratchpad

  for (byte i=0; i<9; i++) {
    b[i] = ds.read();
  }
  if (b[2] == PATa && b[3] == PATb && OneWire::crc8(b, 8) == b[8]) {
    temps[t].raw = b[1] << 8 | b[0];
    temps[t].valid = true;
    validtemps++;
  } else {
    temps[t].valid = false;
  }
}

/*
float raw_to_celsius(int16_t raw)
{
  float c = raw / 16.0f;
  return c;  
}
*/

void read_temps(void)
{
  static byte t;
  static enum { stopped, converting, reading } status = stopped;
  static uint32_t timeStamp;
  static uint8_t passes;

  switch (status) {
    case stopped: 
      if ((now - timeStamp) >= 1000) {
        start_conversion();
        timeStamp = now;
        status = converting;
        passes = 0;
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
        if (!temps[t].valid) {
          read_temp(t);
          t++;
          break;
        }
        t++;
      }
      if (t >= ntemps) {
        t = 0;
        passes++;
      }
      if (validtemps >= ntemps || passes > 3) {
        if (validtemps > 0) {
          hasNewTemps = true;
        }
        status = stopped;
        //timeStamp = now;
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
          hthum_raw = (buf[2] << 8 | buf[3]);
          hthum = hthum_raw / 10.0;
          httmp_raw = ((buf[4] & 0x7f) << 8 | buf[5]);
          if (buf[4] & 0x80) httmp_raw = -httmp_raw;
          httmp = httmp_raw / 10.0;
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
        ht_1w_hum_raw = ((buf[0] << 8) | buf[1]);
        ht_1w_hum = ht_1w_hum_raw / 10.0;
        ht_1w_temp_raw = (((buf[2] & 0x7f) << 8) | buf[3]);
        if (buf[2] & 0x80) ht_1w_temp_raw = -ht_1w_temp_raw;
        ht_1w_temp = ht_1w_temp_raw / 10.0;
        hasNewHT1w = true;
        //Serial.print(timeout);
      }
      status = stopped;
      timeStamp = now;
      break;
  }
}

#define ACT_FAN 0
#define NACTUATORS 1
int8_t actuators[NACTUATORS];

void received_actuator_data(uint8_t actuator, int8_t data)
{
  if (actuator < NACTUATORS) {
    actuators[actuator] = data;
  }
}

void verify_comm(void)
{
  msg m;
  if(msg_get_msg(&m)) {
    uint8_t type = msg_getbyte(&m);
    if (type == '1') {
      uint8_t actuator;
      int8_t data;
      actuator = msg_getbyte(&m);
      data = msg_getbyte(&m);
      received_actuator_data(actuator, data);
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
  pinMode(PIN_FAN_ON, OUTPUT);
  pinMode(PIN_FAN_SPEED, OUTPUT);
  actuators[ACT_FAN] = 1;
  
  Wire.begin();
//pinMode(13, OUTPUT);
  msg_begin();

  //for (i = 0; i < N; i++)
  //  dht[i].begin();

  find_temps();
  //delay(2000);
  temps_begin();
}

void loop() {
  static timestamp_t last_fan_timestamp = 0;
  bool hasNewFan = false;

  now = millis();

  read_temps();
  read_ht();
  read_ht_1w();

  verify_comm();

/*
  {
    static timestamp_t l = 0;
    if (now-l > 2327) {
      static int t=0;
      if (msg_start()) {
        msg_putbyte('i');
        msg_putbyte(t);
        for (byte i=0; i<8; i++) {
          msg_putbyte(temps[t].addr[i]);
        }
      }
      if (msg_end()) {
        t++;
        if (t>=ntemps) t=0;
      }
      l=now;
    }
  }
*/

  switch (actuators[ACT_FAN]) {
    case 0:  // off
      digitalWrite(PIN_FAN_ON, LOW);
      digitalWrite(PIN_FAN_SPEED, LOW);
      break;
    case 2: // fast
      digitalWrite(PIN_FAN_ON, HIGH);
      digitalWrite(PIN_FAN_SPEED, HIGH);
      break;
    default: // slow
      digitalWrite(PIN_FAN_ON, HIGH);
      digitalWrite(PIN_FAN_SPEED, LOW);
      break;
  }

  if (now - last_fan_timestamp > 60000) {
    hasNewFan = true;
    last_fan_timestamp = now;
  }
  uint8_t nsensors = 0;
  if (hasNewTemps) nsensors += validtemps;
  if (hasNewHT) nsensors += 2;
  if (hasNewHT1w) nsensors += 2;
  if (hasNewFan) nsensors += 1;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (hasNewHT) {
      msg_putbyte(0);
      msg_putword(hthum_raw);
      msg_putbyte(1);
      msg_putword(httmp_raw);
    }

    if (hasNewHT1w) {
      msg_putbyte(2);
      msg_putword(ht_1w_hum_raw);
      msg_putbyte(3);
      msg_putword(ht_1w_temp_raw);
    }
    if (hasNewFan) {
      msg_putbyte(100);
      msg_putword(actuators[ACT_FAN]);
    }
    if (hasNewTemps) {
      for (byte i = 0; i < ntemps; i++) {
        if (temps[i].valid) {
          msg_putbyte(4+i);
          msg_putword(temps[i].raw);
        }
      }
    }
    if (msg_end()) {
      hasNewTemps = false;
      hasNewHT = false;
      hasNewHT1w = false;
    }
  }

}
