//#define NODE_ID '2'  // this is the Q2 node
extern const uint8_t node_id = 0xf6;

#include "clab_msg.h"

uint32_t now;

class HT1w      // AM2301 etc HT sensor 1-wire protocol
{
public:
  HT1w();
  ~HT1w();

  void begin(uint8_t dataPin, uint32_t period, uint32_t delay = 0, uint8_t powerPin = -1);
  void verify(void);
  bool hasNewData(void);
  void dataIsOld(void);
  int16_t rawTemperature(void);
  int16_t rawHumidity(void);

private:
  uint8_t _data_pin;
  uint8_t _power_pin;
  uint32_t _period;
  int16_t _raw_humidity;
  int16_t _raw_temperature;
  bool _has_new_data;

  enum { stopped, waiting, waking, reading } _status;
  timestamp_t _timestamp;
};

HT1w::HT1w() {}
HT1w::~HT1w() {}

void HT1w::begin(uint8_t dataPin, uint32_t period, uint32_t delay = 0, uint8_t powerPin = -1)
{
  _data_pin = dataPin;
  _power_pin = powerPin;
  pinMode(_data_pin, INPUT_PULLUP);
  if (_power_pin != -1) pinMode(_power_pin, OUTPUT);
  _period = period;
  _status = stopped;
  _timestamp = delay;
}

void HT1w::verify(void)
{
  switch (_status) {
    case stopped:
      if (now - _timestamp >= _period) {
        // power the sensor up
        if (_power_pin != -1) digitalWrite(_power_pin, HIGH);
        _status = waiting;
        _timestamp = now;
      }
      break;
    case waiting:
      if (now - _timestamp > 2500) {
        // it should be well, let's wake it and read the values
        pinMode(_data_pin, OUTPUT);
        digitalWrite(_data_pin, LOW);
        _status = waking;
      }
      break;    
    case waking:
      if (now - _timestamp > 2502) {
        // it should be awaken, let's read
        _status = reading;
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
      pinMode(_data_pin, INPUT_PULLUP);
      while (timeout && digitalRead(_data_pin) == LOW) timeout--;
      // the sensor should put the pin on low and then on high, for ~80us each
      while (timeout && digitalRead(_data_pin) == HIGH) timeout--;
      while (timeout && digitalRead(_data_pin) == LOW) timeout--;
      // now it should put it on low to start the first bit
      while (timeout && digitalRead(_data_pin) == HIGH) timeout--;

      // read 5 bytes
      for (int i=0; timeout && i<5; i++) {
        byte b; // accumulate the bits here before putting in buf
        // for each byte, we need 8 bits
        for (byte bit=0; bit<8; bit++) {
          byte count_low = 0, count_high = 0;
          b <<= 1;
          while (timeout && digitalRead(_data_pin) == LOW) {
            count_low += 3;
            timeout--;
          }
          while (timeout && digitalRead(_data_pin) == HIGH) {
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
        _raw_humidity = ((buf[0] << 8) | buf[1]);
        //_humidity = _raw_humidity / 10.0;
        _raw_temperature = (((buf[2] & 0x7f) << 8) | buf[3]);
        if (buf[2] & 0x80) _raw_temperature = -_raw_temperature;
        //_temperature = _raw_temperature / 10.0;
        _has_new_data = true;
      }
      _status = stopped;
      if (_power_pin != -1) digitalWrite(_power_pin, LOW);
      break;
  }
}

bool HT1w::hasNewData(void)
{
  return _has_new_data;
}

void HT1w::dataIsOld(void)
{
  _has_new_data = false;
}

int16_t HT1w::rawTemperature(void)
{
  return _raw_temperature;
}

int16_t HT1w::rawHumidity(void)
{
  return _raw_humidity;
}

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

HT1w q2_ambient_sensor;
HT1w q2_vent_sensor;

void setup() {
  q2_ambient_sensor.begin(4, 20000, 0, 5);
  q2_vent_sensor.begin(7, 20000, 3000);
  msg_begin();
}

void loop() {
  now = millis();

  q2_ambient_sensor.verify();
  q2_vent_sensor.verify();
  heartbeat_verify();

  verify_comm();

  uint8_t nsensors = 0;
  if (q2_ambient_sensor.hasNewData()) nsensors += 2;
  if (q2_vent_sensor.hasNewData()) nsensors += 2;
  if (hasNewHeartbeat) nsensors += 1;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (q2_ambient_sensor.hasNewData()) {
      msg_putbyte(0);
      msg_putword(q2_ambient_sensor.rawHumidity());
      msg_putbyte(1);
      msg_putword(q2_ambient_sensor.rawTemperature());
    }
    if (q2_vent_sensor.hasNewData()) {
      msg_putbyte(2);
      msg_putword(q2_vent_sensor.rawHumidity());
      msg_putbyte(3);
      msg_putword(q2_vent_sensor.rawTemperature());
    }
    if (hasNewHeartbeat) {
      msg_putbyte(255);
      msg_putword(heartbeat_value);
    }
    if (msg_end()) {
      if (q2_ambient_sensor.hasNewData()) q2_ambient_sensor.dataIsOld();
      if (q2_vent_sensor.hasNewData()) q2_vent_sensor.dataIsOld();
      hasNewHeartbeat = false;
    }
  }

}
