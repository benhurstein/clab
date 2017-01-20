//#define NODE_ID '3'  // this is the Q3 node
extern const uint8_t node_id = 0xf4;

#include "clab_msg.h"

  uint32_t now;

  byte ht1w_pin = 4;
  byte ht1w_energy_pin = 5;
  float ht1w_hum;
  int16_t ht1w_hum_raw;
  float ht1w_temp;
  int16_t ht1w_temp_raw;
  bool hasNewHT1w = false;

void ht1w_begin(void)
{
  pinMode(ht1w_energy_pin, OUTPUT);
  digitalWrite(ht1w_energy_pin, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
}

void read_ht1w(void)
{
  static enum { stopped, waiting, waking, reading } status = stopped;
  static uint32_t timeStamp;
  static bool first = true;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 24000) {
        // energize the sensor
        digitalWrite(ht1w_energy_pin, HIGH);
        digitalWrite(6, HIGH);
        first = true;
        status = waiting;
        timeStamp = now;
      }
      break;
    case waiting:
      if (now - timeStamp >= 3000) {
        // wake the sensor up
        pinMode(ht1w_pin, OUTPUT);
        digitalWrite(ht1w_pin, LOW);
        timeStamp = now;
        status = waking;
      }
      break;
    case waking:
      if (now - timeStamp > 2) {
        // it should be awaken, let's read
        status = reading;
        //timeStamp = now;
      }
      break;
    case reading:
      byte buf[5] = {1,2,3,4,5};
      // max number of times to read the pin value
      // so that in case of comm error we don't get stuck
      // value must be changed for faster processor
      uint16_t timeout = 10000;

      // time critical -- cannot be interrupted
      noInterrupts();
      // let the sensor be in control
      pinMode(ht1w_pin, INPUT_PULLUP);
      while (timeout && digitalRead(ht1w_pin) == LOW) timeout--;
      // the sensor should put the pin on low and then on high, for ~80us each
      while (timeout && digitalRead(ht1w_pin) == HIGH) timeout--;
      while (timeout && digitalRead(ht1w_pin) == LOW) timeout--;
      // now it should put it on low to start the first bit
      while (timeout && digitalRead(ht1w_pin) == HIGH) timeout--;

      // read 5 bytes
      for (int i=0; timeout && i<5; i++) {
        byte b; // accumulate the bits here before putting in buf
        // for each byte, we need 8 bits
        for (byte bit=0; bit<8; bit++) {
          byte count_low = 0, count_high = 0;
          b <<= 1;
          while (timeout && digitalRead(ht1w_pin) == LOW) {
            count_low += 3;
            timeout--;
          }
          while (timeout && digitalRead(ht1w_pin) == HIGH) {
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
      if (first) {
        first = false;
        status = waiting;
      } else {
        if (timeout && buf[4] == ((buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF)) {
          ht1w_hum_raw = ((buf[0] << 8) | buf[1]);
          ht1w_hum = ht1w_hum_raw / 10.0;
          ht1w_temp_raw = (((buf[2] & 0x7f) << 8) | buf[3]);
          if (buf[2] & 0x80) ht1w_temp_raw = -ht1w_temp_raw;
          ht1w_temp = ht1w_temp_raw / 10.0;
          hasNewHT1w = true;
          //Serial.print(timeout);
        }
        status = stopped;
        //timeStamp = now;
        digitalWrite(ht1w_energy_pin, LOW);
        digitalWrite(6, LOW);
      }
      break;
  }
}

#if 0
  byte ht1w_pin = 4;
  byte ht1w_energy_pin = 5;
  int16_t ht1w_hum_raw;
  int16_t ht1w_temp_raw;
  bool hasNewHT1w = false;

void ht1w_begin(void)
{
  pinMode(ht1w_energy_pin, OUTPUT);
  digitalWrite(ht1w_energy_pin, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
}

void read_ht1w(void)
{
  static enum { stopped, waiting, waking, reading } status = stopped;
  static uint32_t timeStamp;
  static int8_t nread = 0;
  static bool read_ok;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 22500) {
        // energize the sensor
        digitalWrite(ht1w_energy_pin, HIGH);
        digitalWrite(6, HIGH);
        nread = 0;
        read_ok = false;
        status = waiting;
        timeStamp = now;
      }
      break;
    case waiting:
      if (now - timeStamp >= 2500) {
        // wake the sensor up
        pinMode(ht1w_pin, OUTPUT);
        digitalWrite(ht1w_pin, LOW);
        timeStamp = now;
        status = waking;
      }
      break;
    case waking:
      if (now - timeStamp > 2) {
        // it should be awaken, let's read
        status = reading;
        //timeStamp = now;
      }
      break;
    case reading:
      byte buf[5] = {1,2,3,4,5};
      // max number of times to read the pin value
      // so that in case of comm error we don't get stuck
      // value must be changed for faster processor
      uint16_t timeout = 10000;

      // time critical -- cannot be interrupted
      noInterrupts();
      // let the sensor be in control
      pinMode(ht1w_pin, INPUT_PULLUP);
      while (timeout && digitalRead(ht1w_pin) == LOW) timeout--;
      // the sensor should put the pin on low and then on high, for ~80us each
      while (timeout && digitalRead(ht1w_pin) == HIGH) timeout--;
      while (timeout && digitalRead(ht1w_pin) == LOW) timeout--;
      // now it should put it on low to start the first bit
      while (timeout && digitalRead(ht1w_pin) == HIGH) timeout--;

      // read 5 bytes
      for (int i=0; timeout && i<5; i++) {
        byte b; // accumulate the bits here before putting in buf
        // for each byte, we need 8 bits
        for (byte bit=0; bit<8; bit++) {
          byte count_low = 0, count_high = 0;
          b <<= 1;
          while (timeout && digitalRead(ht1w_pin) == LOW) {
            count_low += 3;
            timeout--;
          }
          while (timeout && digitalRead(ht1w_pin) == HIGH) {
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
      if (timeout && buf[4] == ((buf[0] + buf[1] + buf[2] + buf[3]) & 0xFF)) {
        ht1w_hum_raw = ((buf[0] << 8) | buf[1]);
        ht1w_temp_raw = (((buf[2] & 0x7f) << 8) | buf[3]);
        if (buf[2] & 0x80) ht1w_temp_raw = -ht1w_temp_raw;
        read_ok = true;
      }
      if (nread<2) {
        nread++;
        status = waiting; // let's try again
      } else {
        hasNewHT1w = read_ok;
        status = stopped;
        digitalWrite(ht1w_energy_pin, LOW);
        digitalWrite(6, LOW);
      }
      break;
  }
}
#endif

bool hasNewHeartbeat = false;
byte heartbeat_value = 0;

void heartbeat_begin(void)
{
  pinMode(13, OUTPUT);
}

void heartbeat_verify(void)
{
  int x = now/125;
  digitalWrite(13, x&1 && !(x&12) && x&16 ? HIGH : LOW);

  byte v = (x&0x40) == 0;
  if (v != heartbeat_value) {
    heartbeat_value = v;
    hasNewHeartbeat = true;
  }
}

#define NACTUATORS 0
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
  ht1w_begin();
  msg_begin();
}

void loop() {
  now = millis();

  read_ht1w();
  heartbeat_verify();

  verify_comm();

  uint8_t nsensors = 0;
  if (hasNewHT1w) nsensors += 2;
  if (hasNewHeartbeat) nsensors += 1;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (hasNewHT1w) {
      msg_putbyte(0);
      msg_putword(ht1w_hum_raw);
      msg_putbyte(1);
      msg_putword(ht1w_temp_raw);
    }
    if (hasNewHeartbeat) {
      msg_putbyte(255);
      msg_putword(heartbeat_value);
    }
    if (msg_end()) {
      hasNewHT1w = false;
      hasNewHeartbeat = false;
    }
  }

}
