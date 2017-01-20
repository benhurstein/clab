
  uint32_t now;

  byte ht1w_pin = 4;
  byte ht1w_energy_pin = 5;
  float ht1w_hum;
  int16_t ht1w_hum_raw;
  float ht1w_temp;
  int16_t ht1w_temp_raw;
  bool hasNewHT1w = false;
  
  byte ht2w_pin = 7;
  byte ht2w_energy_pin = 8;
  float ht2w_hum;
  int16_t ht2w_hum_raw;
  float ht2w_temp;
  int16_t ht2w_temp_raw;
  bool hasNewHT2w = false;

void ht1w_begin(void)
{
  pinMode(ht1w_energy_pin, OUTPUT);
  digitalWrite(ht1w_energy_pin, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, LOW);
}

void ht2w_begin(void)
{
  pinMode(ht2w_energy_pin, OUTPUT);
  digitalWrite(ht2w_energy_pin, LOW);
}

void read_ht1w(void)
{
  static enum { stopped, waiting, waking, reading } status = stopped;
  static uint32_t timeStamp = -4000;
  static bool first = true;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 4000) {
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
void read_ht2w(void)
{
  static enum { stopped, waiting, waking, reading } status = stopped;
  static uint32_t timeStamp;
  static bool first = true;

  switch (status) {
    case stopped:
      if (now - timeStamp >= 24000) {
        // energize the sensor
        digitalWrite(ht2w_energy_pin, HIGH);
        first = true;
        status = waiting;
        timeStamp = now;
      }
      break;
    case waiting:
      if (now - timeStamp >= 3000) {
        // wake the sensor up
        pinMode(ht2w_pin, OUTPUT);
        digitalWrite(ht2w_pin, LOW);
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
      pinMode(ht2w_pin, INPUT_PULLUP);
      while (timeout && digitalRead(ht2w_pin) == LOW) timeout--;
      // the sensor should put the pin on low and then on high, for ~80us each
      while (timeout && digitalRead(ht2w_pin) == HIGH) timeout--;
      while (timeout && digitalRead(ht2w_pin) == LOW) timeout--;
      // now it should put it on low to start the first bit
      while (timeout && digitalRead(ht2w_pin) == HIGH) timeout--;

      // read 5 bytes
      for (int i=0; timeout && i<5; i++) {
        byte b; // accumulate the bits here before putting in buf
        // for each byte, we need 8 bits
        for (byte bit=0; bit<8; bit++) {
          byte count_low = 0, count_high = 0;
          b <<= 1;
          while (timeout && digitalRead(ht2w_pin) == LOW) {
            count_low += 3;
            timeout--;
          }
          while (timeout && digitalRead(ht2w_pin) == HIGH) {
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
          ht2w_hum_raw = ((buf[0] << 8) | buf[1]);
          ht2w_hum = ht2w_hum_raw / 10.0;
          ht2w_temp_raw = (((buf[2] & 0x7f) << 8) | buf[3]);
          if (buf[2] & 0x80) ht2w_temp_raw = -ht2w_temp_raw;
          ht2w_temp = ht2w_temp_raw / 10.0;
          hasNewHT2w = true;
          //Serial.print(timeout);
        }
        status = stopped;
        //timeStamp = now;
        digitalWrite(ht2w_energy_pin, LOW);
      }
      break;
  }
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

/*
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
*/

bool tapiscando = false;
byte npisca;
long proxima_inversao;
bool estado = false;

void verify_pisca()
{
  if (!tapiscando) return;
  if (proxima_inversao > now) return;
  estado = !estado;
  if (estado) {
    digitalWrite(13, HIGH);
    if (npisca == 0) {
      proxima_inversao = now + 30;
    } else {
      proxima_inversao = now + 125;
    }
  } else {
    digitalWrite(13, LOW);
    if (npisca <= 1) {
      proxima_inversao = now + 600;
      tapiscando = false;
    } else {
      proxima_inversao = now + 250;
      npisca--;
    }
  }
}

void setup() {
  ht1w_begin();
  ht2w_begin();
  heartbeat_begin();
  //msg_begin();
  Serial.begin(115200);
  Serial.println("Start");
}

int d = 0;

void loop() {
  now = millis();

  read_ht1w();
  read_ht2w();
  //heartbeat_verify();

  //verify_comm();
  verify_pisca();
  if (!tapiscando) {
    if (d == 0) { npisca = ht1w_temp_raw / 100; d = 1; }
    else if (d == 1) { npisca = (ht1w_temp_raw / 10) % 10; d = 2; }
    else { npisca = 0; d = 0; }
    tapiscando = true;
  }

  if (hasNewHT1w) {
    Serial.print(now/1000);
    Serial.print(" H1 ");
    Serial.print(ht1w_hum_raw);
    Serial.print(" T1 ");
    Serial.println(ht1w_temp_raw);
    hasNewHT1w = false;
  }
  if (hasNewHT2w) {
    Serial.print(now/1000);
    Serial.print(" H2 ");
    Serial.print(ht2w_hum_raw);
    Serial.print(" T2 ");
    Serial.println(ht2w_temp_raw);
    hasNewHT2w = false;
  }
  /*
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
*/
}
