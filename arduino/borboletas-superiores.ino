//#define NODE_ID 'D'  // this is the air damper controller node (sup)
extern const uint8_t node_id = 0xfd;

#include "clab_msg.h"

timestamp_t now;

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

#include <Servo.h>

Servo servo;

#define NSERVOS 10
// SS ES SN SJ Q1N Q1S Q2 Q3 B1 B2
byte servo_pin[NSERVOS] = { 11, 12, A4, A5, 4, A1, 3, A2, 2, A3 };
byte servo_closed_val[NSERVOS] = { 140, 150, 140, 140, 160, 155, 145, 150, 145, 135};
byte servo_open_val[NSERVOS] = { 40, 55, 50, 50, 60, 55, 55, 45, 50, 45 };
byte servo_pos[NSERVOS];
byte servo_want[NSERVOS];

#define SERVO_ON_TIME 2000
#define SERVO_OFF_TIME 500
#define SERVO_TIME (SERVO_ON_TIME + SERVO_OFF_TIME)

void servo_begin(void)
{
}

void actuate_servo(void)
{
  static byte current_servo = 0;
  static bool moving;
  static timestamp_t timestamp;
  timestamp_t current_time;
  current_time = now - timestamp;

  if (current_time > SERVO_TIME) {
    if (servo_pos[current_servo] != servo_want[current_servo]) {
      servo_pos[current_servo] = servo_want[current_servo];
      int val, delta_val;
      val = servo_closed_val[current_servo];
      delta_val = val - servo_open_val[current_servo];
      val -= servo_pos[current_servo] * delta_val / 100;
      servo.write(val);
      servo.attach(servo_pin[current_servo]);
      moving = true;
      timestamp = now;
    } else {
      current_servo = (current_servo + 1) % NSERVOS;
    }
  } else {
    if (moving && current_time > SERVO_ON_TIME) {
      servo.detach();
      moving = false;
    }
  }
}


#define PIN_RELAY_REC_SUP  5
#define PIN_RELAY_REC_INF A0
#define PIN_RELAY_REC_ON   6
// ta sobrando um relay no pino 7
#define PIN_RELAY_VENT_ON  9
#define PIN_RELAY_VENT_VEL 8
#define PIN_RELAY_BOMBA_FRIO 10
#define PIN_RELAY_BOMBA_QUENTE 13

enum { closed, open } damper_rec_sup_state, damper_rec_inf_state;
#define DAMPER_DELAY SECONDS(20)
static timestamp_t damper_timestamp;

enum { stopped, low, high } main_fan_speed;

enum { off, on } bomba_cond_frio_state, bomba_cond_quente_state;

void relay_begin(void)
{
  digitalWrite(PIN_RELAY_REC_SUP, HIGH);
  digitalWrite(PIN_RELAY_REC_INF, HIGH);
  digitalWrite(PIN_RELAY_REC_ON, HIGH);
  digitalWrite(PIN_RELAY_VENT_VEL, HIGH);
  digitalWrite(PIN_RELAY_VENT_ON, HIGH);
  digitalWrite(PIN_RELAY_BOMBA_FRIO, HIGH);
  digitalWrite(PIN_RELAY_BOMBA_QUENTE, HIGH);
  pinMode(PIN_RELAY_REC_SUP, OUTPUT);
  pinMode(PIN_RELAY_REC_INF, OUTPUT);
  pinMode(PIN_RELAY_REC_ON, OUTPUT);
  pinMode(PIN_RELAY_VENT_VEL, OUTPUT);
  pinMode(PIN_RELAY_VENT_ON, OUTPUT);
  pinMode(PIN_RELAY_BOMBA_FRIO, OUTPUT);
  pinMode(PIN_RELAY_BOMBA_QUENTE, OUTPUT);
}

void actuate_relay(void)
{
  if (now - damper_timestamp < DAMPER_DELAY) {
    digitalWrite(PIN_RELAY_REC_SUP, damper_rec_sup_state == closed ? HIGH : LOW);
    digitalWrite(PIN_RELAY_REC_INF, damper_rec_inf_state == closed ? HIGH : LOW);
    digitalWrite(PIN_RELAY_REC_ON, LOW);
  } else {
    digitalWrite(PIN_RELAY_REC_ON, HIGH);
    digitalWrite(PIN_RELAY_REC_SUP, HIGH);
    digitalWrite(PIN_RELAY_REC_INF, HIGH);
  }

  if (main_fan_speed == stopped) {
    digitalWrite(PIN_RELAY_VENT_ON, HIGH);
    digitalWrite(PIN_RELAY_VENT_VEL, HIGH);
  } else {
    digitalWrite(PIN_RELAY_VENT_VEL, main_fan_speed == low ? HIGH : LOW);
    digitalWrite(PIN_RELAY_VENT_ON, LOW);
  }

  if (bomba_cond_frio_state == off) {
    digitalWrite(PIN_RELAY_BOMBA_FRIO, HIGH);
  } else {
    digitalWrite(PIN_RELAY_BOMBA_FRIO, LOW);
  }
  if (bomba_cond_quente_state == off) {
    digitalWrite(PIN_RELAY_BOMBA_QUENTE, HIGH);
  } else {
    digitalWrite(PIN_RELAY_BOMBA_QUENTE, LOW);
  }
}

#define ACT_BORBOLETA_ESTAR_SUL    0
#define ACT_BORBOLETA_ESCRITORIO   1
#define ACT_BORBOLETA_ESTAR_NORTE  2
#define ACT_BORBOLETA_JANTAR       3
#define ACT_BORBOLETA_Q1_NORTE     4
#define ACT_BORBOLETA_Q1_SUL       5
#define ACT_BORBOLETA_Q2           6
#define ACT_BORBOLETA_Q3           7
#define ACT_BORBOLETA_B1           8
#define ACT_BORBOLETA_B2           9
#define ACT_BORBOLETA_RECIRC_INF  10
#define ACT_BORBOLETA_RECIRC_SUP  11
#define ACT_VENT_PRINCIPAL        12
#define ACT_BOMBA_COND_FRIO       13
#define ACT_BOMBA_COND_QUENTE     14
#define NACTUATORS                15

void received_actuator_data(uint8_t actuator, int8_t data)
{
  if (actuator < NSERVOS) {
    servo_want[actuator] = data > 100 ? 100 : data;
  } else if (actuator == ACT_BORBOLETA_RECIRC_INF) {
    damper_rec_inf_state = data == 0 ? closed : open;
    damper_timestamp = now;
  } else if (actuator == ACT_BORBOLETA_RECIRC_SUP) {
    damper_rec_sup_state = data == 0 ? closed : open;
    damper_timestamp = now;
  } else if (actuator == ACT_VENT_PRINCIPAL) {
    main_fan_speed = data == 0 ? stopped : data == 1 ? low : high;
  } else if (actuator == ACT_BOMBA_COND_FRIO) {
    bomba_cond_frio_state = data == 0 ? off : on;
  } else if (actuator == ACT_BOMBA_COND_QUENTE) {
    bomba_cond_quente_state = data == 0 ? off : on;
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
  servo_begin();
  relay_begin();
}

void loop() {
  now = millis();

  verify_comm();
  actuate_servo();
  actuate_relay();
}

