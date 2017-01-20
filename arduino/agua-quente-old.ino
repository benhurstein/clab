#include <OneWire.h>
#include <TM1638.h>

//
// Controle da agua quente
//
// Faz o seguinte:
// - controla o aquecimento da agua de consumo (liga/desliga as
//   bombas que fazem circular agua dos reservatorios de agua
//   quente no trocador de calor que aquece a agua que circula 
//   nos 4 distribuidores da casa (B1, B2, Coz, Lav);
// - controla a pressao da agua (abre/fecha a valvula que conecta
//   a agua da corsan direto na saida d'agua da caixa)
// - mantem quente a agua nos canos ate os distribuidores (liga/
//   desliga a bomba d'agua que recircula a agua entre o trocador
//   de calor e os distribuidores)
// Para isso, le dados de:
// - vazao de agua nos distribuidores (so estao ligados os de agua
//   quente, falta os de agua fria)
// - vazao de agua entre reservatorios e trocador (ainda nao ligado)
// - temperatura de entrada/saida de agua quente e aquecida do
//   trocador (ainda nao ligado)
// Funcionamento por enquanto:
// - quando tem alguma vazao em algum circuito de agua quente, liga
//   a bomba dos reservatorios
// - quando a vazao de agua quante para B1 e B2 for superior a um
//   patamar, abre a valvula para aumentar a pressao da agua
// - de vez em quando, liga a bomba de recirculacao por algum tempo
// Como deve ser:
// - deve controlar a vazao da agua do reservatorio segundo o 
//   consumo de agua quente, usando as duas bombas, prevendo a
//   vazao necessaria pelo consumo, temperatura da agua do 
//   reservatorio, temperatura da agua de entrada, temperatura
//   desejada para agua de consumo; deve corrigir dinamicamente a
//   vazao verificando a temperatura real da agua de consumo.
// - deve controlar a recirculacao conforme a temperatura da agua
//   nos distribuidores (falta medidores de temperatura nos 
//   distribuidores)
// - controlar a valvula de acordo com a pressao da agua nos 
//   distribuidores (falta os sensores de pressao nos
//   distribuidores)
//

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

// se a vazao em baixa pressao for maior que isso, abre a valvula
#define MAX_FLOW_LOW_PRESSURE 12
// se a vazao em alta pressao for inferior a isso, fecha a valvula
#define MIN_FLOW_HIGH_PRESSURE 10

// de quanto em quanto tempo ligar a bomba de circulacao
#define RECIRCULATION_PUMP_PERIOD MINUTES(100)
// por quanto tempo ligar a cada vez
#define RECIRCULATION_PUMP_ON_TIME MINUTES(5)

// tempo que leva a valvula para abrir ou fechar
#define VALVE_CHANGE_TIME SECONDS(5)

// tempo minimo para deixar a bomba ligada
#define MIN_PUMP_ON_TIME SECONDS(15)

// tempo minimo para deixar a valvula aberta
#define MIN_VALVE_OPEN_TIME SECONDS(15)

// Estado do sistema
//{
  // medidores de vazao

  // conta os pulsos dos 2 primeiros por interrupcao
  volatile unsigned flow_count_int[2]; // conta as interrupcoes do sensor
  unsigned flow_count[4]; // conta os 4 por sw
  byte last_flow_state;
  volatile byte *flowInputPortRegister;
  unsigned last_flow_count_int[2];
  unsigned last_flow_count[4];
  long last_flow_count_timestamp = 0;
  float flow_int[2];
  float flow[4]; // vazao da agua (ints por segundo)

  byte buttons; // keys from TM1638

  typedef enum {off, on} pumpState;
  pumpState hotPumpState;
  long hotPumpTimestamp;

  pumpState recirculationPumpState;
  long recirculationPumpTimestamp;

  enum {closed, open} valveState;
  long valveTimestamp;

  long now;
//}


// pinos
// entradas digitais
#define PIN_FLOW_HOT_1 2
#define PIN_FLOW_HOT_2 3
#define PIN_FLOW_HOT_3 4
#define PIN_FLOW_HOT_4 5
#define PIN_FLOW_COLD_1 6
#define PIN_FLOW_COLD_2 7
#define PIN_FLOW_COLD_3 8
#define PIN_FLOW_COLD_4 9

#define PIN_FLOW_TANK A0

#define PIN_RECIRCULATION_PUMP 10
#define PIN_TANK_SMALLPUMP 1
#define PIN_TANK_BIGPUMP 13

#define PIN_VALVE_A 11
#define PIN_VALVE_B 12


TM1638 disp(A2, A1, A3, true, 1);


// trata uma interrupcao do sendor de vazao
void flow_sensor_ISR0(void)
{
  flow_count_int[0]++;
}
void flow_sensor_ISR1(void)
{
  flow_count_int[1]++;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_FLOW_HOT_1, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_2, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_3, INPUT_PULLUP);
  pinMode(PIN_FLOW_HOT_4, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_1, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_2, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_3, INPUT_PULLUP);
  pinMode(PIN_FLOW_COLD_4, INPUT_PULLUP);
  attachInterrupt(0, flow_sensor_ISR0, CHANGE);
  attachInterrupt(1, flow_sensor_ISR1, CHANGE);
  flowInputPortRegister = portInputRegister(digitalPinToPort(PIN_FLOW_HOT_1));

  pinMode(PIN_TANK_SMALLPUMP, OUTPUT);
  digitalWrite(PIN_TANK_SMALLPUMP, LOW);
  pinMode(PIN_TANK_BIGPUMP, OUTPUT);
  digitalWrite(PIN_TANK_BIGPUMP, LOW);
  pinMode(PIN_RECIRCULATION_PUMP, OUTPUT);
  digitalWrite(PIN_RECIRCULATION_PUMP, LOW);

  pinMode(PIN_VALVE_A, OUTPUT);
  digitalWrite(PIN_VALVE_A, LOW);
  pinMode(PIN_VALVE_B, OUTPUT);
  digitalWrite(PIN_VALVE_B, LOW);

  recirculationPumpTimestamp = -RECIRCULATION_PUMP_PERIOD + MINUTES(1);

  disp.setDisplayToString("Start");
  
  delay(1000);

}

void display_status(void)
{
  byte t = (now / 2500) % 4;
  char s[9];
  switch(t) {
    case 0:
      sprintf(s, "a %3d%3d", (int)(flow_int[0]*10), (int)(flow[0]*10));
      break;
    case 1:
      sprintf(s, "b %3d%3d", (int)(flow_int[1]*10), (int)(flow[1]*10));
      break;
    case 2:
      sprintf(s, "cd%3d%3d", (int)(flow[2]*10), (int)(flow[3]*10));
      break;
    case 3:
      sprintf(s, "%s%s%s", valveState==open?"Open":
                           valveState==closed?"Clos":"Unk ",
                           hotPumpState==on?"On":"Of",
                           recirculationPumpState==on?"On":"Of");
      break;
  }
  disp.setDisplayToString(s);
//  Serial.println(s);
}

void read_flow(void)
{
  byte flow_state = *flowInputPortRegister;
  byte flow_state_diff = flow_state ^ last_flow_state;
  if (flow_state_diff&0x04) flow_count[0]++;
  if (flow_state_diff&0x08) flow_count[1]++;
  if (flow_state_diff&0x10) flow_count[2]++;
  if (flow_state_diff&0x20) flow_count[3]++;
  last_flow_state = flow_state;

  unsigned long delta = now - last_flow_count_timestamp;
  if (delta >= 500) {
    byte i;
    for (i=0; i<2; i++) {
      unsigned current_flow_count = flow_count_int[i]; // tem que desabilitar ints?
      flow_int[i] = (float)(current_flow_count - last_flow_count_int[i]) / delta * 1000;
      last_flow_count_int[i] = current_flow_count;
    }
    for (i=0; i<4; i++) {
      flow[i] = (float)(flow_count[i] - last_flow_count[i]) / delta * 1000;
      last_flow_count[i] = flow_count[i];
    }
    last_flow_count_timestamp = now;
  }
}

void read_buttons(void)
{
  buttons = disp.getButtons();
}

void read_sensors(void)
{
  read_flow();
  read_buttons();
}

void verify_status(void)
{
  if (hotPumpState == off) {
    if (flow[0] + flow[1] + flow[2] + flow[3] > 0) {
      hotPumpState = on;
      hotPumpTimestamp = now;
    }
  }
  if (hotPumpState == on && ((now - hotPumpTimestamp) > MIN_PUMP_ON_TIME)) {
    if (flow[0] + flow[1] + flow[2] + flow[3] == 0) {
      hotPumpState = off;
      hotPumpTimestamp = now;
    }
  }

  if (recirculationPumpState == off) {
    if ((now - recirculationPumpTimestamp) >= RECIRCULATION_PUMP_PERIOD) {
      recirculationPumpState = on;
      recirculationPumpTimestamp = now;
    }
  }
  if (recirculationPumpState == on) {
    if ((now - recirculationPumpTimestamp) >= RECIRCULATION_PUMP_ON_TIME) {
      recirculationPumpState = off;
      recirculationPumpTimestamp = now;
    }
  }

  if (valveState == closed) {
    if ((flow[0] > MAX_FLOW_LOW_PRESSURE)
        || (flow[1] > MAX_FLOW_LOW_PRESSURE)) {
      valveState = open;
      valveTimestamp = now;
    }
  }
  if (valveState == open && ((now - valveTimestamp) > MIN_VALVE_OPEN_TIME)) {
    if ((flow[0] < MIN_FLOW_HIGH_PRESSURE)
        && (flow[1] < MIN_FLOW_HIGH_PRESSURE)) {
      valveState = closed;
      valveTimestamp = now;
    }
  }
}

void change_actuators(void)
{
  if (hotPumpState == on) {
    digitalWrite(PIN_TANK_BIGPUMP, HIGH);
  }
  if (hotPumpState == off) {
    digitalWrite(PIN_TANK_BIGPUMP, LOW);
  }

  if (recirculationPumpState == on) {
    digitalWrite(PIN_RECIRCULATION_PUMP, HIGH);
  }
  if (recirculationPumpState == off) {
    digitalWrite(PIN_RECIRCULATION_PUMP, LOW);
  }

  if ((now - valveTimestamp) < VALVE_CHANGE_TIME) {
    if (valveState == open) {
      digitalWrite(PIN_VALVE_A, HIGH);
      digitalWrite(PIN_VALVE_B, LOW);
    }
    if (valveState == closed) {
      digitalWrite(PIN_VALVE_A, LOW);
      digitalWrite(PIN_VALVE_B, HIGH);
    }
  } else {
    digitalWrite(PIN_VALVE_A, LOW);
    digitalWrite(PIN_VALVE_B, LOW);
  }

}

void loop() {
  byte t;
  now = millis();

  read_sensors();
  verify_status();
  change_actuators();
  display_status();
}
