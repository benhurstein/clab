//#define NODE_ID 'V'  // this is the fan controller node
extern const uint8_t node_id = 0xf3;

#include "clab_msg.h"

timestamp_t now;

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

#define PIN_FAN_IN                 5
#define PIN_FAN_OUT                6
#define PIN_FAN_RECIRCULATION_SUP  9
#define PIN_FAN_RECIRCULATION_INF 10

byte fan_in_value = 0;
byte fan_out_value = 0;
byte fan_rec_sup_value = 0;
byte fan_rec_inf_value = 0;

void fan_begin(void)
{
  digitalWrite(PIN_FAN_IN, HIGH);
  digitalWrite(PIN_FAN_OUT, HIGH);
  digitalWrite(PIN_FAN_RECIRCULATION_SUP, HIGH);
  digitalWrite(PIN_FAN_RECIRCULATION_INF, HIGH);

  pinMode(PIN_FAN_IN, OUTPUT);
  pinMode(PIN_FAN_OUT, OUTPUT);
  pinMode(PIN_FAN_RECIRCULATION_SUP, OUTPUT);
  pinMode(PIN_FAN_RECIRCULATION_INF, OUTPUT);
}

void actuate_fan(void)
{
  analogWrite(PIN_FAN_IN, map(fan_in_value, 0, 100, 255, 0));
  analogWrite(PIN_FAN_OUT, map(fan_out_value, 0, 100, 255, 0));
  analogWrite(PIN_FAN_RECIRCULATION_SUP, map(fan_rec_sup_value, 0, 100, 255, 75));
  analogWrite(PIN_FAN_RECIRCULATION_INF, map(fan_rec_inf_value, 0, 100, 255, 100));
}

#define ACT_FAN_IN       0
#define ACT_FAN_OUT      1
#define ACT_FAN_REC_SUP  2
#define ACT_FAN_REC_INF  3
#define NACTUATORS       4

void received_actuator_data(uint8_t actuator, int8_t data)
{
  data = constrain(data, 0, 100);
  switch (actuator) {
    case ACT_FAN_IN:
      fan_in_value = data;
      break;
    case ACT_FAN_OUT:
      fan_out_value = data;
      break;
    case ACT_FAN_REC_SUP:
      fan_rec_sup_value = data;
      break;
    case ACT_FAN_REC_INF:
      fan_rec_inf_value = data;
      break;
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
  msg_begin();
  fan_begin();
}

void loop() {
  now = millis();
  verify_comm();
  actuate_fan();
}
