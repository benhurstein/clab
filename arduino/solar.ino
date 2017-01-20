//#define NODE_ID 'S'  // this is the Solar node
extern const uint8_t node_id = 0xf5;

//
// Controle do coletor solar
//
// Liga/desliga os motores que fazem circular agua no coletor solar
// Faz essa agua circular pelo reservatorio de agua quante ou pelo
//   trocador de calor da piscina
// Coleta temperaturas nos coletores (5), na saida dos motores (1) e
//   no retorno dos coletores (1)
// Coleta vazao d'agua nos coletores
//
// Sao 3 circuitos d'agua:
//   - um que circula pelos coletores, e contem as bombas 
//     (2 bombas em paralelo que empurram agua de um dos
//     outros circuitos aos coletores) e os sensores (temp, vazao); 
//   - outro que circula pelo reservatorio;
//   - outro que circula pelo trocador de calor da piscina.
//
// Liga/desliga os motores dependendo da coleta de energia solar
//   pelos coletores: sem energia, bombas desligadas, pouca energia,
//   bomba pequena ligada; bastante energia, bomba grande ligada.
//   fallback para bomba pequena (12V) em caso de falta de energia
//   eletrica
// Desvia a agua aquecida para o reservatorio ou a piscina
//   dependendo da energia armazenada no reservatorio


#include "clab_msg.h"

#include <OneWire.h>

timestamp_t now;

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

typedef enum {closed, open, unknown} valve_state_t;

#define NFLOWS 2
  volatile uint8_t flow_count_int[2]; // conta as interrupcoes do sensor
  uint8_t last_flow_count_int[2];
#ifdef SEM_INT
  uint8_t flow_count[NFLOWS]; // conta por sw
  uint8_t last_flow_count[NFLOWS];
#else
  volatile uint8_t flow_count[NFLOWS]; // conta por sw
  uint8_t last_flow_count[NFLOWS];
#endif

  volatile uint8_t *flowInputPortRegisterA;
  volatile uint8_t *flowInputPortRegisterB;
  uint8_t last_flow_stateA;
  uint8_t last_flow_stateB;

  timestamp_t last_flow_count_timestamp = 0;

  unsigned flow_int[2];
  unsigned flow[NFLOWS]; // vazao da agua (16*ints por segundo)
  bool hasNewFlow = false;

#define PIN_FLOW_SOLAR      2  // 
#define PIN_FLOW_HEAT_PUMP A3  //

// trata uma interrupcao do sensor de vazao
void flow_sensor_ISR0(void)
{
  flow_count_int[0]++;
}
/*
void flow_sensor_ISR1(void)
{
  flow_count_int[1]++;
}
*/

#ifndef SEM_INT
ISR(TIMER1_COMPA_vect)
{
  uint8_t flow_state_diff;
  uint8_t flow_stateA = *flowInputPortRegisterA;
  uint8_t flow_stateB = *flowInputPortRegisterB;

  flow_state_diff = flow_stateA ^ last_flow_stateA;
  if (flow_state_diff&0x04) flow_count[0]++; //D2
  last_flow_stateA = flow_stateA;

  flow_state_diff = flow_stateB ^ last_flow_stateB;
  if (flow_state_diff&0x08) flow_count[1]++; //A3
  last_flow_stateB = flow_stateB;
}
#endif

void flow_begin()
{
  pinMode(PIN_FLOW_SOLAR, INPUT_PULLUP);
  pinMode(PIN_FLOW_HEAT_PUMP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SOLAR), flow_sensor_ISR0, CHANGE);
  flowInputPortRegisterA = portInputRegister(digitalPinToPort(PIN_FLOW_SOLAR));
  flowInputPortRegisterB = portInputRegister(digitalPinToPort(PIN_FLOW_HEAT_PUMP));
#ifndef SEM_INT
  // usa o timer1 para gerar interrupcoes a 1kHz para ler os
  // sinais dos sensores de fluxo. Com isso, perde-se o PWM
  // nos pinos 9 e 10, e tambem suporte a servos.
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 16000; // 16MHz/16k = ints a 1kHz
  TCCR1B |= (1 << WGM12); // modo CTC - int e zera qdo chega em OCR
  TCCR1B |= (1 << CS10); // ctr a 16MHz
  TIMSK1 |= (1 << OCIE1A); // habilita int TC - timer compare
  interrupts();
#endif
}

void read_flow(void)
{
#ifdef SEM_INT
  uint8_t flow_state_diff;
  uint8_t flow_stateA = *flowInputPortRegisterA;
  uint8_t flow_stateB = *flowInputPortRegisterB;

  flow_state_diff = flow_stateA ^ last_flow_stateA;
  if (flow_state_diff&0x04) flow_count[0]++; //D2
  last_flow_stateA = flow_stateA;

  flow_state_diff = flow_stateB ^ last_flow_stateB;
  if (flow_state_diff&0x08) flow_count[1]++; //A3
  last_flow_stateB = flow_stateB;
#endif

//  timestamp_t delta = now - last_flow_count_timestamp;
//  if (delta >= 1000) {
if (hasNewFlow) return;
    byte i;
    uint8_t count;
    uint8_t current_flow_count;
    for (i=0; i<2; i++) {
      current_flow_count = flow_count_int[i];
      count = current_flow_count - last_flow_count_int[i];
      flow_int[i] += count/* * 16000L / delta*/;
      last_flow_count_int[i] = current_flow_count;
    }
    for (i=0; i<NFLOWS; i++) {
      current_flow_count = flow_count[i];
      count = current_flow_count - last_flow_count[i];
      flow[i] += count/* * 16000L / delta*/;
      last_flow_count[i] = current_flow_count;
    }
  timestamp_t delta = now - last_flow_count_timestamp;
  if (delta >= 1000) {
    //flow_int[0] = 1000L * (long)flow_int[0] / delta;
    //flow[0] = 1000L * (long)flow[0] / delta;
    //flow[1] = 1000L * (long)flow[1] / delta;
    last_flow_count_timestamp += 1000;//= now;
    hasNewFlow = true;
  }
}

void reset_flow(void)
{
  if (hasNewFlow) {
    flow_int[0] = flow[0] = flow[1] = 0;
    hasNewFlow = false;
  }
}

// TANKVALVE - se aberta, água sai do tanque para o coletor solar
#define PIN_TANKVALVE_A 6
#define PIN_TANKVALVE_B 7
#define TANKVALVE_CHANGE_TIME SECONDS(4)

// POOLVALVE - se aberta, água vem do TC da piscina para o coletor
#define PIN_POOLVALVE_A 5
#define PIN_POOLVALVE_B A0
#define POOLVALVE_CHANGE_TIME SECONDS(4)


  valve_state_t tankvalveState, poolvalveState;
  timestamp_t tankvalveTimestamp, poolvalveTimestamp;

void valve_begin(void)
{
  pinMode(PIN_TANKVALVE_A, OUTPUT);
  digitalWrite(PIN_TANKVALVE_A, LOW);
  pinMode(PIN_TANKVALVE_B, OUTPUT);
  digitalWrite(PIN_TANKVALVE_B, LOW);

  tankvalveState = unknown;
  tankvalveTimestamp = 0;

  pinMode(PIN_POOLVALVE_A, OUTPUT);
  digitalWrite(PIN_POOLVALVE_A, LOW);
  pinMode(PIN_POOLVALVE_B, OUTPUT);
  digitalWrite(PIN_POOLVALVE_B, LOW);

  poolvalveState = unknown;
  poolvalveTimestamp = 0;
}

void verify_valve(void)
{
}

void actuate_valve(void)
{
  if ((now - tankvalveTimestamp) < TANKVALVE_CHANGE_TIME) {
    if (tankvalveState == open) {
      digitalWrite(PIN_TANKVALVE_A, HIGH);
      digitalWrite(PIN_TANKVALVE_B, LOW);
    }
    if (tankvalveState == closed) {
      digitalWrite(PIN_TANKVALVE_A, LOW);
      digitalWrite(PIN_TANKVALVE_B, HIGH);
    }
  } else {
    digitalWrite(PIN_TANKVALVE_A, LOW);
    digitalWrite(PIN_TANKVALVE_B, LOW);
  }

  if ((now - poolvalveTimestamp) < POOLVALVE_CHANGE_TIME) {
    if (poolvalveState == open) {
      digitalWrite(PIN_POOLVALVE_A, HIGH);
      digitalWrite(PIN_POOLVALVE_B, LOW);
    }
    if (poolvalveState == closed) {
      digitalWrite(PIN_POOLVALVE_A, LOW);
      digitalWrite(PIN_POOLVALVE_B, HIGH);
    }
  } else {
    digitalWrite(PIN_POOLVALVE_A, LOW);
    digitalWrite(PIN_POOLVALVE_B, LOW);
  }
}

#define PIN_SOLAR_SMALLPUMP 11
#define PIN_SOLAR_BIGPUMP   12

// tempo minimo para deixar a bomba ligada
#define MIN_PUMP_ON_TIME SECONDS(15)

uint8_t solar_smallpump_value = 0;
uint8_t solar_bigpump_value = 0;
timestamp_t solarPumpTimestamp = 0;

void tank_pump_begin(void)
{
  pinMode(PIN_SOLAR_SMALLPUMP, OUTPUT);
  pinMode(PIN_SOLAR_BIGPUMP, OUTPUT);
}

void verify_solar_pump(void)
{
  if (solar_bigpump_value == 0) {
    if (0) {
      solar_bigpump_value = 255;
      solarPumpTimestamp = now;
    }
  }
  if (solar_bigpump_value > 0 && ((now - solarPumpTimestamp) > MIN_PUMP_ON_TIME)) {
    if (0) {
      solar_bigpump_value = 0;
      solarPumpTimestamp = now;
    }
  }
}

void actuate_solar_pump(void)
{
  analogWrite(PIN_SOLAR_SMALLPUMP, solar_smallpump_value);
  if (solar_bigpump_value == 0) {
    digitalWrite(PIN_SOLAR_BIGPUMP, LOW);
  } else {
    digitalWrite(PIN_SOLAR_BIGPUMP, HIGH);
  }
}




// Com todos os sensores no mesmo pino, nao funcionou.
// Coloquei alguns em um pino outros no outro.
#define PIN_TEMP1 9
#define PIN_TEMP2 8

// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  byte addr[8];  // endereco do sensor
  int16_t raw;   // dado cru da temperatura
  bool valid;    // validade do dado em raw
} temp;

  temp temps[] = {
    { { 0x28, 0xff, 0x70, 0xe1, 0x03, 0x15, 0x02, 0xa8 }, 0, false }, //anTQ-TM
    { { 0x28, 0xff, 0x84, 0x6a, 0x04, 0x15, 0x03, 0x78 }, 0, false }, //retBC
    { { 0x28, 0xff, 0xa4, 0x90, 0x3e, 0x04, 0x00, 0x94 }, 0, false }, //saiAQ
    { { 0x28, 0xff, 0xac, 0x0e, 0x63, 0x14, 0x02, 0x65 }, 0, false }, //TGbx
    { { 0x28, 0xff, 0x02, 0x5f, 0x63, 0x14, 0x02, 0xa3 }, 0, false }, //TQbx
    { { 0x28, 0xff, 0x56, 0x4a, 0x04, 0x15, 0x03, 0xd5 }, 0, false }, //anTrQ
    { { 0x28, 0xff, 0x09, 0x46, 0x63, 0x14, 0x02, 0x78 }, 0, false }, //ambTQTM
    { { 0x28, 0xff, 0x49, 0x29, 0x62, 0x14, 0x03, 0x6e }, 0, false }, //retPisc
    { { 0x28, 0xff, 0x29, 0x1e, 0x63, 0x14, 0x02, 0x44 }, 0, false }, //TGm
    { { 0x28, 0xff, 0x35, 0x4b, 0x3a, 0x04, 0x00, 0xe5 }, 0, false }, //retArQ
    { { 0x28, 0xff, 0x9d, 0x6f, 0x3b, 0x04, 0x00, 0x3d }, 0, false }, //saiArQ
    { { 0x28, 0xff, 0xf3, 0x43, 0x63, 0x14, 0x02, 0xe2 }, 0, false }, //TQm
    { { 0x28, 0xff, 0x5b, 0x31, 0x63, 0x14, 0x02, 0x40 }, 0, false }, //TMbx
    { { 0x28, 0xff, 0xbb, 0x81, 0xa8, 0x15, 0x03, 0x03 }, 0, false }, //ambTGTM
    { { 0x28, 0xff, 0xa7, 0x39, 0x62, 0x14, 0x03, 0x2e }, 0, false }, //TGal
    { { 0x28, 0xff, 0xdf, 0x44, 0x63, 0x14, 0x02, 0xbd }, 0, false }, //TMma
    { { 0x28, 0xff, 0xff, 0x4b, 0x62, 0x14, 0x03, 0x2a }, 0, false }, //TQa
    // SOLAR
    { { 0x28, 0xff, 0x60, 0x6c, 0x04, 0x15, 0x03, 0xb4 }, 0, false }, //sol2
    { { 0x28, 0xff, 0x48, 0x64, 0x04, 0x15, 0x03, 0x6e }, 0, false }, //sol5
    { { 0x28, 0xff, 0x2c, 0x69, 0x04, 0x15, 0x03, 0xfd }, 0, false }, //solret
    { { 0x28, 0xff, 0x2d, 0xcd, 0x04, 0x15, 0x03, 0x97 }, 0, false }, //sol3
    { { 0x28, 0xff, 0xa3, 0xc7, 0x03, 0x15, 0x02, 0xc1 }, 0, false }, //solsai
    { { 0x28, 0xff, 0xbb, 0x4d, 0x04, 0x15, 0x03, 0x65 }, 0, false }, //sol4
    { { 0x28, 0xff, 0x4f, 0x63, 0x04, 0x15, 0x03, 0xb9 }, 0, false }, //sol1

  };
  const byte ntemps = sizeof(temps)/sizeof(temps[0]);
  uint8_t validtemps = 0;
  bool hasNewTemps = false;

OneWire ds1(PIN_TEMP1);
OneWire ds2(PIN_TEMP2);

const byte PATa = 0xac;
const byte PATb = 0x65;

void temps_begin(void)
{
  ds1.reset();
  ds1.write(0xcc); // envia para todos os sensores
  ds1.write(0x4e); // write scratchpad
  ds1.write(PATa); // padrao de bits para conferir nas leituras
  ds1.write(PATb); // outro padrao
  ds1.write(0x7f); // configuracao - conversao de 12 bits

  ds2.reset();
  ds2.write(0xcc); // envia para todos os sensores
  ds2.write(0x4e); // write scratchpad
  ds2.write(PATa); // padrao de bits para conferir nas leituras
  ds2.write(PATb); // outro padrao
  ds2.write(0x7f); // configuracao - conversao de 12 bits
}

/*
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
  //ntemps = t;
}
*/

void start_conversion(void)
{
  ds1.reset();
  ds1.write(0xcc); // envia a todos os sensores
  ds1.write(0x44); // pedido de início de conversão

  ds2.reset();
  ds2.write(0xcc); // envia a todos os sensores
  ds2.write(0x44); // pedido de início de conversão

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

void read_temp1(byte t)
{
  byte b[9];
  
  ds1.reset();
  ds1.select(temps[t].addr);    
  ds1.write(0xBE);         // Read Scratchpad

  for (byte i=0; i<9; i++) {
    b[i] = ds1.read();
  }
  if (b[2] == PATa && b[3] == PATb && OneWire::crc8(b, 8) == b[8]) {
    temps[t].raw = b[1] << 8 | b[0];
    temps[t].valid = true;
    validtemps++;
  } else {
    temps[t].valid = false;
  }
}
void read_temp2(byte t)
{
  byte b[9];
  
  ds2.reset();
  ds2.select(temps[t].addr);    
  ds2.write(0xBE);         // Read Scratchpad

  for (byte i=0; i<9; i++) {
    b[i] = ds2.read();
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
      if ((now - timeStamp) >= 5000) {
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
          if ((passes%2) == 0) read_temp1(t);
          else read_temp2(t);
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


#define PIN_HEATPUMP_ON  A4 // inverse logic, LOW=relay on=h-pump on
#define PIN_HEATPUMP_HOT A5 // inverse logic, LOW=relay on=heating

#define PIN_HEATPUMP_HOTVALVE_A   4
#define PIN_HEATPUMP_HOTVALVE_B  A1
#define PIN_HEATPUMP_COLDVALVE_A 10
#define PIN_HEATPUMP_COLDVALVE_B 13

#define PIN_HEATPUMP_HOTCOLDVALVE_ON A2
#define PIN_HEATPUMP_HOTCOLDVALVE_HOT 3

#define ENABLED 0
#define DISABLED 1
uint8_t heatpump_state = ENABLED;
#define COOLING 0
#define HEATING 1
uint8_t heatpump_mode = COOLING;

valve_state_t heatpump_hotvalve_state = unknown;
valve_state_t heatpump_coldvalve_state = unknown;
timestamp_t heatpump_valve_timestamp = 0;

#define HOTCOLDVALVE_CHANGE_TIME SECONDS(15)
uint8_t heatpump_hotcoldvalve_state = COOLING;
timestamp_t heatpump_hotcoldvalve_timestamp = 0;

void heatpump_begin(void)
{
  digitalWrite(PIN_HEATPUMP_ON, HIGH);
  digitalWrite(PIN_HEATPUMP_HOT, HIGH);
  digitalWrite(PIN_HEATPUMP_HOTVALVE_A, LOW);
  digitalWrite(PIN_HEATPUMP_HOTVALVE_B, LOW);
  digitalWrite(PIN_HEATPUMP_COLDVALVE_A, LOW);
  digitalWrite(PIN_HEATPUMP_COLDVALVE_B, LOW);
  digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_ON, LOW);
  digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_HOT, LOW);
  pinMode(PIN_HEATPUMP_ON, OUTPUT);
  pinMode(PIN_HEATPUMP_HOT, OUTPUT);
  pinMode(PIN_HEATPUMP_HOTVALVE_A, OUTPUT);
  pinMode(PIN_HEATPUMP_HOTVALVE_B, OUTPUT);
  pinMode(PIN_HEATPUMP_COLDVALVE_A, OUTPUT);
  pinMode(PIN_HEATPUMP_COLDVALVE_B, OUTPUT);
  pinMode(PIN_HEATPUMP_HOTCOLDVALVE_ON, OUTPUT);
  pinMode(PIN_HEATPUMP_HOTCOLDVALVE_HOT, OUTPUT);
}

void verify_heatpump(void)
{
}

void actuate_heatpump(void)
{
  if (heatpump_state == DISABLED) {
    digitalWrite(PIN_HEATPUMP_ON, HIGH);
  } else {
    digitalWrite(PIN_HEATPUMP_ON, LOW);
  }
  if (heatpump_mode == HEATING) {
    digitalWrite(PIN_HEATPUMP_HOT, LOW);
  } else {
    digitalWrite(PIN_HEATPUMP_HOT, HIGH);
  }

  if ((now - heatpump_valve_timestamp) < POOLVALVE_CHANGE_TIME) {
    if (heatpump_hotvalve_state == open) {
      digitalWrite(PIN_HEATPUMP_HOTVALVE_A, HIGH);
      digitalWrite(PIN_HEATPUMP_HOTVALVE_B, LOW);
    }
    if (heatpump_hotvalve_state == closed) {
      digitalWrite(PIN_HEATPUMP_HOTVALVE_A, LOW);
      digitalWrite(PIN_HEATPUMP_HOTVALVE_B, HIGH);
    }
    if (heatpump_coldvalve_state == open) {
      digitalWrite(PIN_HEATPUMP_COLDVALVE_A, HIGH);
      digitalWrite(PIN_HEATPUMP_COLDVALVE_B, LOW);
    }
    if (heatpump_coldvalve_state == closed) {
      digitalWrite(PIN_HEATPUMP_COLDVALVE_A, LOW);
      digitalWrite(PIN_HEATPUMP_COLDVALVE_B, HIGH);
    }
  } else {
    digitalWrite(PIN_HEATPUMP_HOTVALVE_A, LOW);
    digitalWrite(PIN_HEATPUMP_HOTVALVE_B, LOW);
    digitalWrite(PIN_HEATPUMP_COLDVALVE_A, LOW);
    digitalWrite(PIN_HEATPUMP_COLDVALVE_B, LOW);
  }

  if ((now - heatpump_hotcoldvalve_timestamp) < HOTCOLDVALVE_CHANGE_TIME) {
    if (heatpump_hotcoldvalve_state == HEATING) {
      digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_HOT, HIGH);
    } else {
      digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_HOT, LOW);
    }
    digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_ON, HIGH);
  } else {
    digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_ON, LOW);
    digitalWrite(PIN_HEATPUMP_HOTCOLDVALVE_HOT, LOW);
  }
}

#define ACT_TANKVALVE             0
#define ACT_POOLVALVE             1
#define ACT_SOLAR_SMALLPUMP       2
#define ACT_SOLAR_BIGPUMP         3
#define ACT_HEATPUMP_STATE        4
#define ACT_HEATPUMP_MODE         5
#define ACT_HEATPUMP_HOTVALVE     6
#define ACT_HEATPUMP_COLDVALVE    7
#define ACT_HEATPUMP_HOTCOLDVALVE 8
#define NACTUATORS                9
int8_t actuators[NACTUATORS];

void received_actuator_data(uint8_t actuator, int8_t data)
{
  if (actuator < NACTUATORS) {
    actuators[actuator] = data;
    switch (actuator) {
      case ACT_TANKVALVE:
        tankvalveState = (data == 0) ? closed : open;
        tankvalveTimestamp = now;
        break;
      case ACT_POOLVALVE:
        poolvalveState = (data == 0) ? closed : open;
        poolvalveTimestamp = now;
        break;
      case ACT_SOLAR_SMALLPUMP:
        solar_smallpump_value = (int)data * 255 / 100;
        break;
      case ACT_SOLAR_BIGPUMP:
        solar_bigpump_value = (int)data * 255 / 100;
        break;
      case ACT_HEATPUMP_STATE:
        heatpump_state = (data == 0) ? DISABLED : ENABLED;
        break;
      case ACT_HEATPUMP_MODE:
        heatpump_mode = (data == 0) ? COOLING : HEATING;
        break;
      case ACT_HEATPUMP_HOTVALVE:
        heatpump_hotvalve_state = (data == 0) ? closed : open;
        heatpump_valve_timestamp = now;
        break;
      case ACT_HEATPUMP_COLDVALVE:
        heatpump_coldvalve_state = (data == 0) ? closed : open;
        heatpump_valve_timestamp = now;
        break;
      case ACT_HEATPUMP_HOTCOLDVALVE:
        heatpump_hotcoldvalve_state = (data == 0) ? COOLING : HEATING;
        heatpump_hotcoldvalve_timestamp = now;
        break;
    }
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
  temps_begin();
  flow_begin();
  valve_begin();
  tank_pump_begin();
  heatpump_begin();
}

float calc_max_collector_temp(void)
{
  float m = -55, t;
  if (temps[23].valid && (t=raw_to_celsius(temps[23].raw)) > m) m = t;
  if (temps[17].valid && (t=raw_to_celsius(temps[17].raw)) > m) m = t;
  if (temps[20].valid && (t=raw_to_celsius(temps[20].raw)) > m) m = t;
  if (temps[22].valid && (t=raw_to_celsius(temps[22].raw)) > m) m = t;
  if (temps[18].valid && (t=raw_to_celsius(temps[18].raw)) > m) m = t;
  return m == -55 ? 70 : m;
}

float calc_tank_temp(void)
{
  float t = 0;
  int n = 0;
  if (temps[11].valid) { t += raw_to_celsius(temps[11].raw); n++; }
  if (temps[16].valid) { t += raw_to_celsius(temps[16].raw); n++; }
  return n == 0 ? 65 : t/n;
}

float calc_solar_temp_gain(void)
{
  if (temps[19].valid && temps[21].valid) {
    return raw_to_celsius(temps[19].raw) - raw_to_celsius(temps[21].raw);
  }
  if (temps[18].valid && temps[23].valid) {
    return raw_to_celsius(temps[18].raw) - raw_to_celsius(temps[23].raw);
  }
  return 3;
}

void select_solar_circuit(float tank_temp)
{
  if (tank_temp < 56) {
    tankvalveState = open;
    tankvalveTimestamp = now;
    poolvalveState = closed;
    poolvalveTimestamp = now;
  } else {
    tankvalveState = closed;
    tankvalveTimestamp = now;
    poolvalveState = open;
    poolvalveTimestamp = now;
  }  
}

void verify_state(void)
{
  static timestamp_t last = 0;
  if (now - last < 5000) return; // to have time to read temps at start

  if (solar_bigpump_value == 0) {
    float max_collector_temp = calc_max_collector_temp();
    float tank_temp = calc_tank_temp();
    if (max_collector_temp > tank_temp+2) {
      solar_bigpump_value = 255;
      last = now;
      select_solar_circuit(tank_temp);
    }
    return;
  }
  if (now-last < MINUTES(20)) {
    return;
  }
  last = now;
  float solar_temp_gain = calc_solar_temp_gain();
  if (solar_temp_gain < 1) {
    solar_bigpump_value = 0;
    tankvalveState = closed;
    tankvalveTimestamp = now;
    poolvalveState = closed;
    poolvalveTimestamp = now;
  } else {
    select_solar_circuit(calc_tank_temp());
  }
}

void loop() {
  static timestamp_t last_fan_timestamp = 0;
  static bool hasNewFan = false;

  now = millis();

  read_temps();
  read_flow();
  verify_valve();
  verify_solar_pump();

  //verify_heatpump();

  if (hasNewTemps) verify_state();

  actuate_valve();
  actuate_solar_pump();
  actuate_heatpump();

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

/*
  static timestamp_t l=0;
  static uint16_t n1=0, n2=0, n3=0;
  uint16_t dt = now-l;
  n1++;
  if (dt != 0) {
    l=now;
    n2++;
    if (dt>n3) n3=dt;
  }
  if (n1>50000) {
  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(3);
    msg_putbyte(108);
    msg_putword(n1);
    msg_putbyte(109);
    msg_putword(n2);
    msg_putbyte(110);
    msg_putword(n3);
    if (msg_end()) {
      n1=n2=n3=0;
    }
  }
  }
*/

  if (now - last_fan_timestamp > 15000) {
    hasNewFan = true;
    last_fan_timestamp = now;
  }
  uint8_t nsensors = 0;
  if (hasNewTemps) nsensors += validtemps;
  if (hasNewFan) nsensors += 9;
  if (hasNewFlow) nsensors += NFLOWS;
  if (nsensors == 0) return;

  if (msg_start()) {
    msg_putbyte('2');
    msg_putbyte(nsensors);
    if (hasNewFan) {
      msg_putbyte(100);
      msg_putword(tankvalveState == closed ? 0 : 1);
      msg_putbyte(101);
      msg_putword(poolvalveState == closed ? 0 : 1);
      msg_putbyte(102);
      msg_putword(solar_smallpump_value);
      msg_putbyte(103);
      msg_putword(solar_bigpump_value);
      msg_putbyte(104);
      msg_putword(heatpump_state == DISABLED ? 0 : 1);
      msg_putbyte(105);
      msg_putword(heatpump_mode == COOLING ? 0 : 1);
      msg_putbyte(106);
      msg_putword(heatpump_hotvalve_state == closed ? 0 : 1);
      msg_putbyte(107);
      msg_putword(heatpump_coldvalve_state == closed ? 0 : 1);
      msg_putbyte(108);
      msg_putword(heatpump_hotcoldvalve_state == COOLING ? 0 : 1);
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
      for (byte i = 0; i < 1; i++) {
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
