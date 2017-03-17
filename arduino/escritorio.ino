//#define NODE_ID 'O'  // this is the office node
extern const uint8_t node_id = 0xf1;

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



#include "SoftwareWire.h"
class HTU21D    // HTU21D HT sensor I2C protocol
// from code by Nathan Seidle, SparkFun
{
public:
  HTU21D(uint8_t sdaPin, uint8_t sclPin);
  void begin(uint32_t period);
  void verify(void);
  bool hasNewData(void);
  void dataIsOld(void);
  uint16_t rawTemperature(void);
  uint16_t rawHumidity(void);
private:
  bool crc_check(uint16_t value, uint8_t crc);
  bool read_measurement(uint16_t *ptr);
  
  SoftwareWire _wire;
  uint32_t _period;
  uint16_t _raw_humidity;
  uint16_t _raw_temperature;
  bool _has_new_data;

  enum { stopped, waitingT, readingT, waitingH, readingH } _status;
  timestamp_t _timestamp;

  const byte HTDU21D_ADDRESS = 0x40;  //Unshifted 7-bit I2C address for the sensor
  const byte TRIGGER_TEMP_MEASURE_HOLD = 0xE3;
  const byte TRIGGER_HUMD_MEASURE_HOLD = 0xE5;
  const byte TRIGGER_TEMP_MEASURE_NOHOLD = 0xF3;
  const byte TRIGGER_HUMD_MEASURE_NOHOLD = 0xF5;
  const byte WRITE_USER_REG = 0xE6;
  const byte READ_USER_REG = 0xE7;
  const byte SOFT_RESET = 0xFE;
};

HTU21D::HTU21D(uint8_t sdaPin, uint8_t sclPin)
: _wire(sdaPin, sclPin)
{
}

void HTU21D::begin(uint32_t period)
{
  _period = period;
  _status = stopped;
}

// from https://github.com/TEConnectivity/HTU21D_Arduino_Library/blob/master/src/htu21d.cpp
bool HTU21D::crc_check(uint16_t value, uint8_t crc)
{
  uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
  uint32_t msb = 0x800000;
  uint32_t mask = 0xFF8000;
  uint32_t result = (uint32_t)value << 8; // Pad with zeros as specified in spec

  while (msb != 0x80) {

    // Check if msb of current value is 1 and apply XOR mask
    if (result & msb)
      result = ((result ^ polynom) & mask) | (result & ~mask);

    // Shift by one
    msb >>= 1;
    mask >>= 1;
    polynom >>= 1;
  }
  return result == crc;
}

bool HTU21D::read_measurement(uint16_t *ptr)
{
  uint8_t msb, lsb, crc;
  uint16_t meas;

  //Comes back in three bytes, data(MSB) / data(LSB) / CRC
  _wire.requestFrom(HTDU21D_ADDRESS, 3);

  if (_wire.available() < 3) {
    return false;
  }

  msb = _wire.read();
  lsb = _wire.read();
  crc = _wire.read();

  meas = ((unsigned int)msb << 8) | lsb;
  if (!crc_check(meas, crc)) {
    return false;
  }

  *ptr = meas & 0xFFFC;
  return true;
}

void HTU21D::verify(void)
{
  switch (_status) {
    case stopped:
      if (now - _timestamp >= _period) {
        _timestamp = now;
        //Request the temperature
        _wire.beginTransmission(HTDU21D_ADDRESS);
        _wire.write(TRIGGER_TEMP_MEASURE_NOHOLD);
        if (_wire.endTransmission() == 0) {
          _status = waitingT;
        }
      }
      break;
    case waitingT:
      if (now - _timestamp > 50) {
        // data should be ready, let's read
        _status = readingT;
      }
      break;
    case readingT:
      if (!read_measurement(&_raw_temperature)) {
        _status = stopped;
        break;
      }

      // temperature is done, let's ask for humidity
      _wire.beginTransmission(HTDU21D_ADDRESS);
      _wire.write(TRIGGER_HUMD_MEASURE_NOHOLD); //Measure humidity with no bus holding
      if (_wire.endTransmission() != 0) {
        _status = stopped;
      } else {
        _status = waitingH;
      }
      break;
    case waitingH:
      // datasheet says it takes up to 16ms,
      // let's make it 50 from start of T reading
      if (now - _timestamp > 100) {
        // data should be ready, let's read
        _status = readingH;
      }
      break;
    case readingH:
      if (!read_measurement(&_raw_humidity)) {
        _status = stopped;
        break;
      }

      _has_new_data = true;
      _status = stopped;
      break;
  }
}

bool HTU21D::hasNewData(void)
{
  return _has_new_data;
}

void HTU21D::dataIsOld(void)
{
  _has_new_data = false;
}

uint16_t HTU21D::rawTemperature(void)
{
  return _raw_temperature;
}

uint16_t HTU21D::rawHumidity(void)
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


HTU21D escr_ambient_sensor(4, 5);
HT1w sala_ventn_sensor;
HT1w sala_vents_sensor;
HT1w escr_vent_sensor;
HT1w escr_wall_sensor;

void setup() {

  escr_ambient_sensor.begin(10000);
  sala_ventn_sensor.begin(6, 20000, 3000);
  sala_vents_sensor.begin(7, 20000, 6000);
  escr_vent_sensor.begin(8, 20000, 9000);
  escr_wall_sensor.begin(9, 20000, 12000);
  msg_begin();
}

void loop() {
  now = millis();

  heartbeat_verify();

  escr_ambient_sensor.verify();
  sala_ventn_sensor.verify();
  sala_vents_sensor.verify();
  escr_vent_sensor.verify();
  escr_wall_sensor.verify();

  verify_comm();

  uint8_t nsensors = 0;
  if (escr_ambient_sensor.hasNewData()) nsensors += 2;
  if (sala_ventn_sensor.hasNewData()) nsensors += 2;
  if (sala_vents_sensor.hasNewData()) nsensors += 2;
  if (escr_vent_sensor.hasNewData()) nsensors += 2;
  if (escr_wall_sensor.hasNewData()) nsensors += 2;
  if (hasNewHeartbeat) nsensors += 1;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (escr_ambient_sensor.hasNewData()) {
      msg_putbyte(0);
      msg_putword(escr_ambient_sensor.rawHumidity());
      msg_putbyte(1);
      msg_putword(escr_ambient_sensor.rawTemperature());
    }
    if (sala_ventn_sensor.hasNewData()) {
      msg_putbyte(2);
      msg_putword(sala_ventn_sensor.rawHumidity());
      msg_putbyte(3);
      msg_putword(sala_ventn_sensor.rawTemperature());
    }
    if (sala_vents_sensor.hasNewData()) {
      msg_putbyte(4);
      msg_putword(sala_vents_sensor.rawHumidity());
      msg_putbyte(5);
      msg_putword(sala_vents_sensor.rawTemperature());
    }
    if (escr_vent_sensor.hasNewData()) {
      msg_putbyte(6);
      msg_putword(escr_vent_sensor.rawHumidity());
      msg_putbyte(7);
      msg_putword(escr_vent_sensor.rawTemperature());
    }
    if (escr_wall_sensor.hasNewData()) {
      msg_putbyte(8);
      msg_putword(escr_wall_sensor.rawHumidity());
      msg_putbyte(9);
      msg_putword(escr_wall_sensor.rawTemperature());
    }
    if (hasNewHeartbeat) {
      msg_putbyte(255);
      msg_putword(heartbeat_value);
    }
    if (msg_end()) {
      if (escr_ambient_sensor.hasNewData()) escr_ambient_sensor.dataIsOld();
      if (sala_ventn_sensor.hasNewData()) sala_ventn_sensor.dataIsOld();
      if (sala_vents_sensor.hasNewData()) sala_vents_sensor.dataIsOld();
      if (escr_vent_sensor.hasNewData()) escr_vent_sensor.dataIsOld();
      if (escr_wall_sensor.hasNewData()) escr_wall_sensor.dataIsOld();
      hasNewHeartbeat = false;
    }
  }
}
