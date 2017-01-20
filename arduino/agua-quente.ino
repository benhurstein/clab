//#define NODE_ID 'W'  // this is the hot water node W
extern const uint8_t node_id = 0xfb;

#include "clab_msg.h"

#include <OneWire.h>

timestamp_t now;

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

#define REMOTECONTROL_TIMEOUT MINUTES(20)
timestamp_t actuatorTimestamp = -REMOTECONTROL_TIMEOUT;


#define NFLOWS 10
  volatile uint8_t tick_count_int[2]; // conta as interrupcoes do sensor
  uint8_t last_tick_count_int[2];
  volatile uint8_t tick_count[NFLOWS];  // conta os por sw
  uint8_t last_tick_count[NFLOWS];

  volatile uint8_t *flowInputPortRegisterH;
  volatile uint8_t *flowInputPortRegisterC;
  uint8_t last_flow_stateH;
  uint8_t last_flow_stateC;

  timestamp_t last_flow_timestamp = 0;

  unsigned flow_int[2];
  unsigned ticks[NFLOWS]; // acumula os ticks aqui
  unsigned flow[NFLOWS]; // vazao da agua (ticks no ultimo segundo)
  bool hasNewFlow = false;

#define PIN_FLOW_HOT_1   2  // AQ B1
#define PIN_FLOW_COLD_2 A3  // AF B2
#define PIN_FLOW_HOT_2   3  // AQ B2
#define PIN_FLOW_COLD_1 A2  // AF B1
#define PIN_FLOW_HOT_3   4  // AQ Coz
#define PIN_FLOW_COLD_3 A1  // AF Coz
#define PIN_FLOW_HOT_4   5  // AQ Serv
#define PIN_FLOW_COLD_4 A0  // AF Serv
#define PIN_FLOW_TANK   A4  // AQ primario do TC
#define PIN_FLOW_MAIN   A5  // AF entrada

// trata uma interrupcao do sensor de vazao
void flow_sensor_ISR0(void)
{
  tick_count_int[0]++;
}
void flow_sensor_ISR1(void)
{
  tick_count_int[1]++;
}

ISR(TIMER1_COMPA_vect)
{
  uint8_t flow_state_diff;
  uint8_t flow_stateH = *flowInputPortRegisterH;
  uint8_t flow_stateC = *flowInputPortRegisterC;

  flow_state_diff = flow_stateH ^ last_flow_stateH;
  if (flow_state_diff&0x04) tick_count[0]++; //D2
  if (flow_state_diff&0x08) tick_count[2]++; //D3
  if (flow_state_diff&0x10) tick_count[4]++; //D4
  if (flow_state_diff&0x20) tick_count[6]++; //D5
//  if (flow_state_diff&0x40) tick_count[8]++; //D6
//  if (flow_state_diff&0x80) tick_count[9]++; //D7
  last_flow_stateH = flow_stateH;

  flow_state_diff = flow_stateC ^ last_flow_stateC;
  if (flow_state_diff&0x20) tick_count[9]++; //A5
  if (flow_state_diff&0x10) tick_count[8]++; //A4
  if (flow_state_diff&0x08) tick_count[3]++; //A3
  if (flow_state_diff&0x04) tick_count[1]++; //A2
  if (flow_state_diff&0x02) tick_count[5]++; //A1
  if (flow_state_diff&0x01) tick_count[7]++; //A0
  last_flow_stateC = flow_stateC;
}

void flow_begin()
{
  pinMode(PIN_FLOW_HOT_1, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_2, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_3, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_4, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_1, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_2, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_3, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_4, INPUT_PULLUP);
  pinMode(PIN_FLOW_TANK, INPUT_PULLUP);
  pinMode(PIN_FLOW_MAIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_HOT_1), flow_sensor_ISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_HOT_2), flow_sensor_ISR1, CHANGE);
  flowInputPortRegisterH = portInputRegister(digitalPinToPort(PIN_FLOW_HOT_1));
  flowInputPortRegisterC = portInputRegister(digitalPinToPort(PIN_FLOW_COLD_1));

  // usa o timer1 para gerar interrupcoes a 1kHz para ler os
  // sinais dos sensores de fluxo. Com isso, perde-se o PWM
  // nos pinos 9 e 10, e tambem suporte a servos.
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 16000; // 16MHz/16k = ints a 1kHz
  TCCR1B |= (1 << WGM12);  // modo CTC - int e zera qdo chega em OCR
  TCCR1B |= (1 << CS10);   // ctr a 16MHz
  TIMSK1 |= (1 << OCIE1A); // habilita int TC - timer compare
  interrupts();
}

void read_flow(void)
{
/*
  uint8_t flow_state_diff;
  uint8_t flow_stateH = *flowInputPortRegisterH;
  uint8_t flow_stateC = *flowInputPortRegisterC;

  flow_state_diff = flow_stateH ^ last_flow_stateH;
  if (flow_state_diff&0x04) tick_count[0]++; //D2
  if (flow_state_diff&0x08) tick_count[2]++; //D3
  if (flow_state_diff&0x10) tick_count[4]++; //D4
  if (flow_state_diff&0x20) tick_count[6]++; //D5
  if (flow_state_diff&0x40) tick_count[8]++; //D6
  if (flow_state_diff&0x80) tick_count[9]++; //D7
  last_flow_stateH = flow_stateH;

  flow_state_diff = flow_stateC ^ last_flow_stateC;
  if (flow_state_diff&0x08) tick_count[3]++; //A3
  if (flow_state_diff&0x04) tick_count[1]++; //A2
  if (flow_state_diff&0x02) tick_count[5]++; //A1
  if (flow_state_diff&0x01) tick_count[7]++; //A0
  last_flow_stateC = flow_stateC;
*/
static int n=0;
if (hasNewFlow) return;
  timestamp_t delta = now - last_flow_timestamp;
  if (delta >= 1000 || (delta>=500&&n==0) || (delta>=1000&&n==1)) {
n++;
    byte i;
    uint8_t count;
    uint8_t current_tick_count;
    for (i=0; i<2; i++) {
      current_tick_count = tick_count_int[i];
      count = current_tick_count - last_tick_count_int[i];
      flow_int[i] += count/* * 16000L / delta*/;
      last_tick_count_int[i] = current_tick_count;
    }
    for (i=0; i<NFLOWS; i++) {
      current_tick_count = tick_count[i];
      count = current_tick_count - last_tick_count[i];
      ticks[i] += count;
      last_tick_count[i] = current_tick_count;
    }
  }
//  timestamp_t delta = now - last_flow_timestamp;
  if (delta >= 1000) {
n=0;
    for (byte i=0; i<2; i++) {
      //flow_int[i] = 1000L * (long)flow_int[i] / delta;
    }
    for (byte i=0; i<NFLOWS; i++) {
      //flow[i] = 1000L * (long)ticks[i] / delta;
      flow[i] = ticks[i];
    }
    last_flow_timestamp += 1000L;//= now;
    hasNewFlow = true;
  }
}

void reset_flow(void)
{
  if (hasNewFlow) {
    byte i;
    for (i=0; i<2; i++) {
      flow_int[i] = 0;
    }
    for (i=0; i<NFLOWS; i++) {
      ticks[i] = 0;
    }
    hasNewFlow = false;
  }
}


#define PIN_VALVE_A 10
#define PIN_VALVE_B 13

// se a vazao em baixa pressao for maior que isso, abre a valvula
#define MAX_FLOW_LOW_PRESSURE  6
// se a vazao em alta pressao for inferior a isso, fecha a valvula
#define MIN_FLOW_HIGH_PRESSURE 5

// tempo minimo para deixar a valvula aberta
#define MIN_VALVE_OPEN_TIME SECONDS(15)

// tempo que leva a valvula para abrir ou fechar
#define VALVE_CHANGE_TIME SECONDS(10)


  enum {closed, open} valveState;
  timestamp_t valveTimestamp;

void valve_begin(void)
{
  pinMode(PIN_VALVE_A, OUTPUT);
  digitalWrite(PIN_VALVE_A, LOW);
  pinMode(PIN_VALVE_B, OUTPUT);
  digitalWrite(PIN_VALVE_B, LOW);

  valveState = closed;
  valveTimestamp = 0;
}

void verify_valve(void)
{
  // do nothing if being controlled remotely
  if (now - actuatorTimestamp < REMOTECONTROL_TIMEOUT) return;

  if (valveState == closed) {
    if ((flow[0] > MAX_FLOW_LOW_PRESSURE)
        || (flow[2] > MAX_FLOW_LOW_PRESSURE)) {
      valveState = open;
      valveTimestamp = now;
    }
  }
  if (valveState == open && ((now - valveTimestamp) > MIN_VALVE_OPEN_TIME)) {
    if ((flow[0]+flow[1] < MIN_FLOW_HIGH_PRESSURE)
        && (flow[2]+flow[3] < MIN_FLOW_HIGH_PRESSURE)) {
      valveState = closed;
      valveTimestamp = now;
    }
  }
}

void actuate_valve(void)
{
  if ((now - valveTimestamp) < VALVE_CHANGE_TIME) {
    if (valveState == open) {
      digitalWrite(PIN_VALVE_A, LOW);
      digitalWrite(PIN_VALVE_B, HIGH);
    }
    if (valveState == closed) {
      digitalWrite(PIN_VALVE_A, HIGH);
      digitalWrite(PIN_VALVE_B, LOW);
    }
  } else {
    digitalWrite(PIN_VALVE_A, LOW);
    digitalWrite(PIN_VALVE_B, LOW);
  }
}

#define PIN_CELLAR_PUMP 11

uint8_t cellar_pump_value = 128;

void cellar_pump_begin(void)
{
  pinMode(PIN_CELLAR_PUMP, OUTPUT);
}

void actuate_cellar_pump(void)
{
  analogWrite(PIN_CELLAR_PUMP, cellar_pump_value);
}






#define PIN_TEMP 9

// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  byte addr[8];  // endereco do sensor
  int16_t raw;   // dado cru da temperatura
  bool valid;    // validade do dado em raw
} temp;

#define FRIA 3
#define AQUECIDA 2
#define QUENTE 1
#define RESFRIADA 0
  temp temps[] = {
    { { 0x28, 0xff, 0x90, 0x0c, 0x2e, 0x04, 0x00, 0xb6 }, 0, false }, // saida do primario
    { { 0x28, 0xff, 0x22, 0x89, 0x14, 0x14, 0x00, 0x7a }, 0, false }, // entrada do primario
    { { 0x28, 0xff, 0x71, 0x46, 0x14, 0x14, 0x00, 0x02 }, 0, false }, // saida do secundario
    { { 0x28, 0xff, 0x4f, 0x04, 0x12, 0x04, 0x00, 0xfe }, 0, false }, // entrada do secundario
  };
  const byte ntemps = sizeof(temps)/sizeof(temps[0]);
  uint8_t validtemps = 0;
  bool hasNewTemps = false;

OneWire ds(PIN_TEMP);

const byte PATa = 0xac;
const byte PATb = 0x65;

void temps_begin(void)
{
  //Wire.begin();
  ds.reset();
  ds.write(0xcc); // envia para todos os sensores
  ds.write(0x4e); // write scratchpad
  ds.write(PATa); // padrao de bits para conferir nas leituras
  ds.write(PATb); // outro padrao
  ds.write(0x7f); // configuracao - conversao de 12 bits
}

/*
void find_temps(void)
{
  int t;
  for (t = 0; t < ntemps; t++) {
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
  //ntemps = t;
}
*/

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

float raw_to_celsius(int16_t raw)
{
  float c = raw / 16.0f;
  return c;  
}

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
      while (t < ntemps) {
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



#define PIN_TANK_SMALLPUMP 6
#define PIN_TANK_BIGPUMP   7

// tempo minimo para deixar a bomba ligada
#define MIN_PUMP_ON_TIME SECONDS(15)

uint8_t tank_smallpump_value = 0;
uint8_t tank_bigpump_value = 0;
timestamp_t tankPumpTimestamp = 0;

void tank_pump_begin(void)
{
  pinMode(PIN_TANK_SMALLPUMP, OUTPUT);
  pinMode(PIN_TANK_BIGPUMP, OUTPUT);
}

int i=0;
void verify_tank_pump(void)
{
/*  
  if (tank_bigpump_value == 0) {
    if (flow[0] + flow[2] + flow[4] + flow[6] > 2) {
      tank_bigpump_value = 255;
      tankPumpTimestamp = now;
    }
  }
  if (tank_bigpump_value > 0 && ((now - tankPumpTimestamp) > MIN_PUMP_ON_TIME)) {
    if (flow[0] + flow[2] + flow[4] + flow[6] == 0) {
      tank_bigpump_value = 0;
      tankPumpTimestamp = now;
    }
  }
  */
  /*
  if (tank_smallpump_value == 0) {
    if (flow[0] + flow[2] + flow[4] + flow[6] > 2) {
      tank_smallpump_value = 150;
      tankPumpTimestamp = now;
    }
  }
  if (tank_smallpump_value > 0 && ((now - tankPumpTimestamp) > MIN_PUMP_ON_TIME)) {
    if (flow[0] + flow[2] + flow[4] + flow[6] == 0) {
      tank_smallpump_value = 0;
      tankPumpTimestamp = now;
    }
  }
  */
  /*
  if ((now-tankPumpTimestamp)<1000) return;
    tankPumpTimestamp = now;
    static float delta_m = 0;
    static float m = 0;
    int tot_fl = flow[0] + flow[2] + flow[4] + flow[6];
    if (tot_fl == 0) {
      tank_bigpump_value = 0;
      tank_smallpump_value = 0;
      m = delta_m = 0;
    } else {
      float tae = raw_to_celsius(temps[FRIA].raw);
      float tat = raw_to_celsius(temps[QUENTE].raw);
      float p;
      p = 5 * (41 - tae) * tot_fl / (tat - tae);
      tank_smallpump_value = constrain(p, 36, 255);
    }
    */
    ///*
  if ((now-tankPumpTimestamp)<1000) return;
    tankPumpTimestamp = now;
    static float t_a = 0;
    static float I = 0;
    int tot_fl = flow[0] + flow[2] + flow[4] + flow[6];
    if (tot_fl == 0) {
      tank_bigpump_value = 0;
      tank_smallpump_value = 0;
      t_a = I = 0;
    } else {
      float kp = 21, ti = 47, ki = kp/ti, td = 0.98, kd = kp*td/2, tt = 7;
      //ad=(2*Td-N*h)/(2*Td+N*h); bd=2*K*N*Td/(2*Td+N*h)
      //D=ad*D-bd*(y-yold)
      float taq = raw_to_celsius(temps[AQUECIDA].raw);
      float e = 41.0 - taq;
      float P, D;
      //P=K*(b*ysp-y)
      P = kp * e;
      I += ki * e;
      if (t_a == 0) D = 0; // da pra setar o I como chute(t, flx)
      else D = - kd * (taq - t_a);
      float v = P + I + D;

      float u = v;
      if (u < 36) u = 36;
      if (u > 255) u = 255;
      tank_smallpump_value = u;

      I += (u-v)/tt;
      t_a = taq;
    }
    //*/
}

void actuate_tank_pump(void)
{
  analogWrite(PIN_TANK_SMALLPUMP, tank_smallpump_value);
  if (tank_bigpump_value == 0) {
    digitalWrite(PIN_TANK_BIGPUMP, LOW);
  } else {
    digitalWrite(PIN_TANK_BIGPUMP, HIGH);
  }
}



#define ACT_VALVE              0
#define ACT_TANK_SMALLPUMP     1
#define ACT_TANK_BIGPUMP       2
#define ACT_RECIRCULATION_PUMP 3
#define ACT_CELLAR_PUMP        4
#define NACTUATORS             5
int8_t actuators[NACTUATORS];

void received_actuator_data(uint8_t actuator, int8_t data)
{
  if (actuator < NACTUATORS) {
    actuators[actuator] = data;
    switch (actuator) {
      case ACT_VALVE:
        actuatorTimestamp = now;
        if (data == 0) { valveState = closed; valveTimestamp = now; }
        if (data == 1) { valveState = open; valveTimestamp = now; }
        break;
      case ACT_TANK_SMALLPUMP:
        tank_smallpump_value = (int)data * 255 / 100;
        tankPumpTimestamp = now;
        break;
      case ACT_TANK_BIGPUMP:
        tank_bigpump_value = (int)data * 255 / 100;
        tankPumpTimestamp = now;
        break;
/*
      case ACT_RECIRCULATION_PUMP:
        recirculation_pump_value = (int)data * 255 / 100;
        break;
*/
      case ACT_CELLAR_PUMP:
        cellar_pump_value = (int)data * 255 / 100;
        break;
    }
  }
}

short debug = -1;
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
debug=(actuator<<8)|data;
      received_actuator_data(actuator, data);
    }
  }
}


void setup() {
  msg_begin();
  temps_begin();
  flow_begin();
  valve_begin();
  cellar_pump_begin();
  tank_pump_begin();
}

void loop() {
  static timestamp_t last_fan_timestamp = 0;
  static bool hasNewFan = false;

  now = millis();

  read_flow();
  read_temps();

  verify_valve();
  verify_tank_pump();

  actuate_valve();
  actuate_cellar_pump();
  actuate_tank_pump();

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

  if (now - last_fan_timestamp > 1000) {
    hasNewFan = true;
    last_fan_timestamp = now;
  }
  uint8_t nsensors = 0;
  if (hasNewTemps) nsensors += validtemps;
  if (hasNewFan) nsensors += 2;
  if (hasNewFlow) nsensors += NFLOWS;
if (debug!=-1) nsensors++;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (hasNewFan) {
      msg_putbyte(100);
      msg_putword(valveState);
      msg_putbyte(101);
      msg_putword(actuators[ACT_CELLAR_PUMP]);
    }
if (debug!=-1) {
      msg_putbyte(200);
      msg_putword(debug);
      debug=-1;
}
    if (hasNewTemps) {
      for (byte i = 0; i < ntemps; i++) {
        if (temps[i].valid) {
          msg_putbyte(i);
          msg_putword(temps[i].raw);
        }
      }
    }
    if (hasNewFlow) {
      for (byte i = 0; i < NFLOWS; i++) {
        msg_putbyte(50+i);
        msg_putword(flow[i]);
      }
      /*
      for (byte i = 0; i < 2; i++) {
        msg_putbyte(50+NFLOWS+i);
        msg_putword(flow_int[i]);
      }
      */
    }
    if (msg_end()) {
      hasNewTemps = false;
      hasNewFan = false;
      reset_flow(); // hasNewFlow = false;
    }
  }
}
