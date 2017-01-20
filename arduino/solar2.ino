#define NODE_ID 'S'  // this is the Solar node

#include <OneWire.h>
#include <TM1638.h>

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

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

// se algum sensor do coletor marcar essa temp ou mais, desespero
#define MAX_SOLAR_TEMP 75
// se algum sensor do coletor marcar mais que essa temp, deve
// ter alguma bomba ligada
#define MAX_STOPPED_TEMP 70
// se algum sensor do coletor marcar mais que essa temperatura
// apos estar com as bombas desligadas por RESTART_DELAY, alguma
// bomba deve ser ligada 
#define MAX_STOPPED_TEMP_RESTART 45
#define RESTART_DELAY MINUTES(30)
// se a agua no tanque estiver acima dessa temp, vai pra piscina
// -- isso ainda nao ta funcionando, nao tem sensor no tanque
#define TANK_HIGH_TEMP 70

// as bombas devem ser ligadas se a temperatura nos painéis
// estiver subindo mais que esse tanto de graus por minuto
#define MAX_STOPPED_TEMP_INCREASE 0.5


// a diferenca de temperatura entre entrada e saida do coletor
// define qual (se alguma) bomba sera ligada
#define MIN_SMALLPUMP_TEMP_GAIN 1.5
#define MAX_SMALLPUMP_TEMP_GAIN 10
#define MIN_BIGPUMP_TEMP_GAIN 3

// as diferencas em energia (a unidade de energia é W)
#define MIN_ENERGY_GAIN 75
#define MAX_SMALLPUMP_ENERGY_GAIN 500
#define MIN_BIGPUMP_ENERGY_GAIN 300

// tempo apos ligar uma bomba ate considerar que esta estavel
#define PUMP_DELAY MINUTES(5) // 5 min


// Dados para sensores de temperatura (devia ter uma classe para isso)
typedef struct {
  int16_t raw;   // dado cru da temperatura
  float celsius; // temperatura (passa-baixa da temperatura lida)
  byte addr[8];  // endereco do sensor
} temp;

#define T_SOL_1   6
#define T_SOL_2   0
#define T_SOL_3   3
#define T_SOL_4   5
#define T_SOL_5   1
#define T_SOL_IN  4
#define T_SOL_OUT 2
#define NTEMPS 7
// identificacao da posicao dos 7 sensores no vetor
byte tr[] = {T_SOL_IN, T_SOL_1, T_SOL_2, T_SOL_3, 
             T_SOL_4, T_SOL_5, T_SOL_OUT};
// Estado do sistema
//{
  // temperatura dos sensores
  temp temps[NTEMPS];
  byte ntemps = 0;
  bool hasNewTemps = false;

  volatile uint8_t int_flow_count = 0; // conta as interrupcoes do sensor
  uint8_t last_int_flow_count;
  uint32_t accumulated_flow_count = 0;
  uint16_t last_flow_count = 0;
  uint32_t last_flow_count_timestamp = 0;
  int flow; // vazao da agua (ints por segundo)

  byte buttons; // keys from TM1638

  enum {stopped, starting, running} pumpState;
  uint32_t pumpStateTimestamp;
  
  enum {none, small, big, both} runningPump; // qual bomba ta ligada
  uint32_t runningPumpTimestamp;

  enum {toAllclosed, allclosed, tank, pool, toTank, toPool, allopen} waterCircuit;
  uint32_t waterCircuitTimestamp;

  typedef enum {unknown, open, closed, opening, closing} valveState;
  valveState tankvalveState;
  uint32_t tankvalveTimestamp;

  valveState poolvalveState;
  uint32_t poolvalveTimestamp;

  uint32_t now;
  
  #define NOLDTEMP 20
  uint16_t accumulated_volume = 0;
  float oldtemp[NOLDTEMP] = {0};
  float accumulated_temperature = 0;
  uint32_t oldtime[NOLDTEMP] = {0};

  float solar_energy_gain;
  float solar_energy_gain_lp = 0; // low pass

  float solar_temp_increase = 0;
  float solar_temp_increase_last = 0;
  uint32_t solar_temp_increase_timestamp = 0;
//}

float highest_collector_temp(void)
{
  byte i;
  float highest = -529;
#define TEST(n)  if (temps[n].celsius > highest) \
                   highest = temps[n].celsius
  TEST(T_SOL_1);
  TEST(T_SOL_2);
  TEST(T_SOL_3);
  TEST(T_SOL_4);
  TEST(T_SOL_5);
#undef TEST
  return highest;
}

float lowest_collector_temp(void)
{
  byte i;
  float lowest = 529;
#define TEST(n)  if (temps[n].celsius < lowest) \
                   lowest = temps[n].celsius
  TEST(T_SOL_1);
  TEST(T_SOL_2);
  TEST(T_SOL_3);
  TEST(T_SOL_4);
  TEST(T_SOL_5);
#undef TEST
  return lowest;
}

// pinos
// entradas digitais
#define PIN_TEMP 4 // temperaturas
#define PIN_FLOW 2 // vazao de agua -- tem que ter int
#define PIN_LED  13 // pisca com a vazao
#define PIN_VALVE_TANK_OPEN 7 // valvula solar-reservatorio
#define PIN_VALVE_TANK_CLOSED 8 // valvula solar-reservatorio

// saidas digitais
#define PIN_SMALLPUMP 5 // bomba de 12V
#define PIN_BIGPUMP 6 // bomba de 220V
//   a valvula do tanque e controlada por 2 reles, precisa 2 pinos.
//   ela nao desliga sozinha, tem que colocar os 2 em LOW depois de
//   alguns segundos.
//   A HIGH B LOW abre  A LOW B HIGH fecha
#define PIN_TANKVALVE_A 9 // valvula solar-reservatorio
#define PIN_TANKVALVE_B 10 // valvula solar-reservatorio
#define TANKVALVE_DELAY SECONDS(4) // tempo de mudanca de estado da valvula
//   a valvula da piscina e controlada por um rele duplo.
//   o motor dela desliga sozinho.
//   so tem um pino (LOW fecha, HIGH abre)
#define PIN_POOLVALVE 11 // valvula solar-piscina
#define POOLVALVE_DELAY SECONDS(4) // tempo de mudanca de estado da valvula

OneWire ds(PIN_TEMP);
TM1638 disp(A2, A1, A3, true, 1);


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
    temps[t].celsius = NAN;
  }
  ntemps = t;
}

void start_conversion(void)
{
    ds.reset();
    //ds.select(temps[t].addr);
    ds.write(0xcc); // envia a todos os sensores
    ds.write(0x44); // pedido de início de conversão
}

float low_pass(float old_value, float new_value)
{
  if (isnan(old_value)) return new_value;
  if (isnan(new_value)) return old_value;
  if (new_value - old_value > 5 || new_value - old_value < -5) return old_value;
  return old_value * 0.4 + 0.6 * new_value;

}

int16_t read_temp(byte t)
{
  byte b0, b1;
  
  ds.reset();
  ds.select(temps[t].addr);    
  ds.write(0xBE);         // Read Scratchpad

  b0 = ds.read();
  b1 = ds.read();

  return b1 << 8 | b0;
}

float raw_to_celsius(int16_t raw)
{
  float c = raw / 16.0f;
  if (c == 85 || c == 0) return NAN;
  if (c < -55 || c > 125) return NAN;
  return c;  
}

void read_temps(void)
{
  static byte t;
  static enum { stopped, converting, reading } status = stopped;
  static uint32_t timeStamp;

  switch (status) {
    case stopped: 
      start_conversion();
      timeStamp = now;
      status = converting;
      break;
    case converting:
      if ((now - timeStamp) >= 750) {
        t = 0;
        status = reading;
      }
      break;
    case reading:
      if (t < ntemps) {
        temps[t].raw = read_temp(t);
        float celsius = raw_to_celsius(temps[t].raw);
        temps[t].celsius = low_pass(temps[t].celsius, celsius);
        t++;
      }
      if (t >= ntemps) {
        hasNewTemps = true;
        status = stopped;
      }
  }
}

// trata uma interrupcao do sensor de vazao
void flow_sensor_ISR(void)
{
  int_flow_count++;
}


// NSerial
#include <HardwareSerial_private.h>

class NHardwareSerial : public HardwareSerial
{
  public:
    void disabletx(void);
    void enabletx(void);
    void begin(unsigned long baud) { begin(baud, SERIAL_8N1); }
    void begin(unsigned long, uint8_t);
};

void NHardwareSerial::disabletx(void)
{
  flush();
  cbi(*_ucsrb, TXEN0); 
  pinMode(1, INPUT);
  //digitalWrite(1, HIGH);
}
void NHardwareSerial::enabletx(void)
{
  sbi(*_ucsrb, TXEN0); 
}
void NHardwareSerial::begin(unsigned long baud, byte config)
{
  // Try u2x mode first
  uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
  *_ucsra = 1 << U2X0;

  // hardcoded exception for 57600 for compatibility with the bootloader
  // shipped with the Duemilanove and previous boards and the firmware
  // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
  // be > 4095, so switch back to non-u2x mode if the baud rate is too
  // low.
  if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
  {
    *_ucsra = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
  *_ubrrh = baud_setting >> 8;
  *_ubrrl = baud_setting;

  _written = false;

  //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
  config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
  *_ucsrc = config;

  sbi(*_ucsrb, RXEN0);
  cbi(*_ucsrb, TXEN0);
  sbi(*_ucsrb, RXCIE0);
  cbi(*_ucsrb, UDRIE0);
}

#define NSerial (*(NHardwareSerial*)&Serial)
// NSerial end

void verify_comm(void)
{
  byte b;
//goto tosozinho;
  if (NSerial.available()) {
    while (NSerial.available()) {
      b = NSerial.read();
      if (b == NODE_ID) {
//tosozinho:
        NSerial.enabletx();
        NSerial.print(NODE_ID);
        if (hasNewTemps) {
          hasNewTemps = false;
          NSerial.print(" ");
          NSerial.print(now);
          for (byte i = 0; i < ntemps; i++) {
            NSerial.print(" ");
            NSerial.print(temps[i].raw);
            NSerial.print(" ");
            NSerial.print(temps[i].celsius);
          }
          NSerial.print(" ");
          NSerial.print(accumulated_flow_count);
          NSerial.print(" ");
          NSerial.print(flow);
          NSerial.print(" ");
          NSerial.print(runningPump);
          NSerial.print(tankvalveState);
          NSerial.print(poolvalveState);
          NSerial.print(" ");
          NSerial.print(solar_energy_gain);
          NSerial.print(" ");
          NSerial.print(accumulated_volume);
          NSerial.print(" ");
          NSerial.print(oldtemp[0]);
          NSerial.print(" ");
          NSerial.print(oldtemp[9]);
          NSerial.print(" ");
          NSerial.print(oldtemp[19]);
          NSerial.print(" ");
          NSerial.print(solar_energy_gain_lp);
        } else {
          NSerial.print("X");
        }
        NSerial.println("");
        NSerial.disabletx();
//return;
      }
    }
  }
}

void setup() {
  NSerial.begin(115200);

  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(0, flow_sensor_ISR, CHANGE);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SMALLPUMP, OUTPUT);
  digitalWrite(PIN_SMALLPUMP, LOW);
  pinMode(PIN_BIGPUMP, OUTPUT);
  digitalWrite(PIN_BIGPUMP, LOW);
  pinMode(PIN_TANKVALVE_A, OUTPUT);
  digitalWrite(PIN_TANKVALVE_A, LOW);
  pinMode(PIN_TANKVALVE_B, OUTPUT);
  digitalWrite(PIN_TANKVALVE_B, LOW);
  pinMode(PIN_POOLVALVE, OUTPUT);
  digitalWrite(PIN_POOLVALVE, LOW);

  // no inicio, ignora a espera para religar a bomba
  pumpState = stopped;
  pumpStateTimestamp = -RESTART_DELAY;

  disp.setDisplayToString("Start");

  find_temps();
  
  delay(1000);

}

void displayTemp(byte t)
{
  char s[15]; // com numeros neg, sprintf pode passar dos 9 dig
  int delta = (temps[T_SOL_OUT].celsius - temps[T_SOL_IN].celsius)*10;
  //sprintf(s, "t %d %4d", t, (int)(temps[tr[t]].celsius*100));
  if (delta < 100) {
    sprintf(s,"%2d%2d%c%3d", 
              (int)(temps[T_SOL_IN].celsius),
              delta,
              t+'a'-1,
              (int)(temps[tr[t]].celsius*10));
  } else {
    sprintf(s,"%2d%c%1d%c%3d", 
              (int)(temps[T_SOL_IN].celsius),
              'a'+delta/100-1,
              (delta/10)%10,
              t+'a'-1,
              (int)(temps[tr[t]].celsius*10));
  }
  disp.setDisplayToString(s);
}

void read_flow(void)
{
  uint8_t current_int_flow_count, delta_int_flow_count;
  current_int_flow_count = int_flow_count;
  delta_int_flow_count = current_int_flow_count - last_int_flow_count;
  last_int_flow_count = current_int_flow_count;
  accumulated_flow_count += delta_int_flow_count;
  uint32_t delta_time = now - last_flow_count_timestamp;
  if (delta_time >= 500) {
    uint16_t current_flow_count, delta_flow_count;
    current_flow_count = (uint16_t)accumulated_flow_count;
    delta_flow_count = current_flow_count - last_flow_count;
    last_flow_count = current_flow_count;
    last_flow_count_timestamp = now;
    // o fluxo é em número de interrupções por segundo
    // divido por 4 pra caber em 16 bits (nao vi delta_flow_count > 150)
    flow = delta_flow_count * (1000/4) / (delta_time/4);

    // cálculo de oldtemps. é um vetor que mantém o valor aproximado da
    // temperatura de entrada da água em vários pontos do circuito solar,
    // usado para calcular o ganho em temperatura (e em energia) da água
    // quando ela chegar de volta no tanque
    // o volume de água em todo o circuito é +- 14000 + 20 * flow, medido
    // interrupções do medidor de fluxo.
    // esse valor é dividido por 20 para os 20 trechos do circuito.
    // é multiplicado por 500 e comparado com 500 para fazer cálculos com int.
    uint16_t current_volume = delta_flow_count * 500 / (700 + flow);

    // gambiarra, apareceu um nan no primeiro teste
    float temp_in = temps[T_SOL_IN].celsius;
    if (isnan(temp_in)) {
      if (accumulated_volume != 0)
        temp_in = accumulated_temperature / accumulated_volume;
      else
        temp_in = 30;
    }

    accumulated_volume += current_volume;
    // o cálculo da temperatura média é provavelmente exagero.
    accumulated_temperature += current_volume * temp_in;
    if (accumulated_volume >= 500) {
      // desloca as temperaturas no vetor, para simular o deslocamento da água
      for (byte t = 1; t < NOLDTEMP; t++) {
        // mistura um pouco as temperaturas, para evitar picos (eles são
        // diluidos na prática)
        if (oldtemp[t-1] == 0) oldtemp[t-1] = oldtemp[t];
        else if (t == NOLDTEMP-1) oldtemp[t-1] = (oldtemp[t-1]+oldtemp[t]) / 2;
        else oldtemp[t-1] = (oldtemp[t-1] + oldtemp[t] + oldtemp[t+1]) / 3;
        oldtime[t-1] = oldtime[t];
      }
      accumulated_volume -= 500;
      float old_accumulated_temperature = accumulated_temperature;
      accumulated_temperature = accumulated_volume * temp_in;
      old_accumulated_temperature -= accumulated_temperature;
      oldtemp[NOLDTEMP-1] = old_accumulated_temperature / 500;
      oldtime[NOLDTEMP-1] = now;
    }
  }
}

void read_buttons(void)
{
  buttons = disp.getButtons();
}

void read_sensors(void)
{
  read_flow();
  read_temps();
  //read_buttons();
}



void verify_status(void)
{
  float solar_temp = highest_collector_temp();
  static uint32_t oldoldtime = 0;
  if (oldoldtime != oldtime[0]) {
    float solar_temp_gain = temps[T_SOL_OUT].celsius
                          - oldtemp[0];
//  solar_energy_gain = solar_temp_gain * flow;
    oldoldtime = oldtime[0];
    solar_energy_gain = solar_temp_gain / (now - oldtime[0]); // C/ms
    solar_energy_gain *= 1000; // C/s
    solar_energy_gain *= 11660; // g.C/s ou cal/s (tem +- 11,66 l no circ solar)
    solar_energy_gain *= 4.18; // J/s ou W (tem +- 4,18 J em 1 cal)
    solar_energy_gain_lp = solar_energy_gain_lp * 0.9 + solar_energy_gain * 0.1;
  }

  if ((now - solar_temp_increase_timestamp) > 15000) {
    if (solar_temp_increase_last == 0)
      solar_temp_increase_last = 0;
    else
      solar_temp_increase = (solar_temp - solar_temp_increase_last) * 4;
    solar_temp_increase_timestamp = now;
    solar_temp_increase_last = solar_temp;
  }

  if (solar_temp >= MAX_SOLAR_TEMP && runningPump != both) {
    if (pumpState == stopped) {
      pumpState = starting;
      pumpStateTimestamp = now;
    }
    runningPump = both;
    runningPumpTimestamp = now;
    waterCircuit = allopen;
    waterCircuitTimestamp = now;
  }
  if (waterCircuit == allopen) {
    if (tankvalveState != opening && tankvalveState != open) {
      tankvalveState = opening;
      tankvalveTimestamp = now;
    }
    if (poolvalveState != opening && poolvalveState != open) {
      poolvalveState = opening;
      poolvalveTimestamp = now;
    }
  }
  if (runningPump == both && solar_temp <= MAX_STOPPED_TEMP_RESTART) {
      runningPump = big;
      runningPumpTimestamp = now;
      waterCircuit = toTank;
      waterCircuitTimestamp = now;    
  }
  
  if (pumpState == stopped) {
    //if (  (solar_temp >= MAX_STOPPED_TEMP)
    //    ||((solar_temp >= MAX_STOPPED_TEMP_RESTART)
    //       && ((now - pumpStateTimestamp) > RESTART_DELAY))) {
    if (solar_temp_increase > MAX_STOPPED_TEMP_INCREASE) {
      pumpState = starting;
      pumpStateTimestamp = now;
      runningPump = small;
      runningPumpTimestamp = now;

      // gambiarra -- estamos sem sensor nos tanques
      // cada vez que liga a bomba faz circular no tanque para
      // ver se precisa esquentar, vai pra piscina depois se nao
      // precisar (da piscina, nao volta pro tanque por enquanto --
      // so depois de desligar as bombas, tipicamente no dia
      // seguinte)
      waterCircuit = toTank;
      waterCircuitTimestamp = now;
    }
  }
  if (pumpState == starting) {
    if ((now - pumpStateTimestamp) > PUMP_DELAY) {
      pumpState = running;
      pumpStateTimestamp = now;
    }
  }
  if (pumpState == running) {
    if (solar_energy_gain_lp < MIN_ENERGY_GAIN) {
      pumpState = stopped;
      pumpStateTimestamp = now;
      runningPump = none;
      runningPumpTimestamp = now;
    }
    if (runningPump == small && solar_energy_gain_lp > MAX_SMALLPUMP_ENERGY_GAIN) {
      // falta testar se tem energia eletrica...
      pumpState = starting;
      pumpStateTimestamp = now;
      runningPump = big;
      runningPumpTimestamp = now;
    }
    if (runningPump == big && solar_energy_gain_lp < MIN_BIGPUMP_ENERGY_GAIN) {
      pumpState = starting;
      pumpStateTimestamp = now;
      runningPump = small;
      runningPumpTimestamp = now;
    }
  }
  // if pump is on and water is not flowing, change pump
  if (pumpState != stopped && flow == 0 && (now - pumpStateTimestamp) > SECONDS(2)) {
    if (runningPump == big) runningPump = small;
    else if (runningPump == small) runningPump = big;
    runningPumpTimestamp = now;
    pumpStateTimestamp = now;
  }
  if (pumpState == stopped) {
    if (waterCircuit != allclosed && waterCircuit != toAllclosed) {
      waterCircuit = toAllclosed;
      waterCircuitTimestamp = now;
    }
  }

  if (waterCircuit == toAllclosed) {
    if (tankvalveState != closing && tankvalveState != closed) {
      tankvalveState = closing;
      tankvalveTimestamp = now;
    }
    if (tankvalveState == closed
        && (poolvalveState != closing && poolvalveState != closed)) {
      poolvalveState = closing;
      poolvalveTimestamp = now;
    }
  }
  if (waterCircuit == toTank) {
    if (tankvalveState != opening && tankvalveState != open) {
      tankvalveState = opening;
      tankvalveTimestamp = now;
    }
    if (tankvalveState == open
        && (poolvalveState != closing && poolvalveState != closed)) {
      poolvalveState = closing;
      poolvalveTimestamp = now;
    }
  }
  if (waterCircuit == tank) {
    if (runningPump == small || runningPump == big) {
      // should get temperature from tank
      if (temps[T_SOL_IN].celsius > TANK_HIGH_TEMP) {
        waterCircuit = toPool;
        waterCircuitTimestamp = now;
      }
    }
  }
  if (waterCircuit == toPool) {
    if (poolvalveState != opening && poolvalveState != open) {
      poolvalveState = opening;
      poolvalveTimestamp = now;
    }
    if (poolvalveState == open
        && (tankvalveState != closing && tankvalveState != closed)) {
      tankvalveState = closing;
      tankvalveTimestamp = now;
    }
  }
  if (waterCircuit == pool) {
    if (runningPump == small || runningPump == big) {
      // TODO: should get temperature from tank
      //if (temps[T_TANK].celsius < TANK_LOW_TEMP) {
      //  waterCircuit = totank;
      //  waterCircuitTimestamp = now;
      //}
    }
  }

  if (tankvalveState == opening) {
    if ((now - tankvalveTimestamp) > TANKVALVE_DELAY) {
      tankvalveState = open;
      tankvalveTimestamp = now;
    }
  }
  if (poolvalveState == opening) {
    if ((now - poolvalveTimestamp) > POOLVALVE_DELAY) {
      poolvalveState = open;
      poolvalveTimestamp = now;
    }
  }
  if (tankvalveState == closing) {
    if ((now - tankvalveTimestamp) > TANKVALVE_DELAY) {
      tankvalveState = closed;
      tankvalveTimestamp = now;
    }
  }
  if (poolvalveState == closing) {
    if ((now - poolvalveTimestamp) > POOLVALVE_DELAY) {
      poolvalveState = closed;
      poolvalveTimestamp = now;
    }
  }

  if (tankvalveState == closed && poolvalveState == closed
      && waterCircuit != allclosed) {
    waterCircuit = allclosed;
    waterCircuitTimestamp = now;
  }
  if (poolvalveState == closed && tankvalveState == open
      && waterCircuit != tank) {
    waterCircuit = tank;
    waterCircuitTimestamp = now;
  }
  if (poolvalveState == open && tankvalveState == closed
      && waterCircuit != pool) {
    waterCircuit = pool;
    waterCircuitTimestamp = now;
  }

  if (waterCircuit == allclosed && runningPump != none) {
    runningPump = none;
    runningPumpTimestamp = now;
  }

/* isso nao funciona: a acao do botao e desativada tao logo se solte
 *  ele, e quando se tira o modulo com os botoes, ele pensa que ta
 *  tudo apertado, e fica bem louco...
 */
 /*
  if ((buttons&1)) {
    if (runningPump == big) {
      runningPump = none;
      runningPumpTimestamp = now;
    }
    if ((runningPump == small) || (runningPump == none)) {
      runningPump = startingbig;
      runningPumpTimestamp = now;
    }
  }
  if ((buttons&2)) {
    if (runningPump == small) {
      runningPump = none;
      runningPumpTimestamp = now;
    }
    if ((runningPump == big) || (runningPump == none)) {
      runningPump = startingsmall;
      runningPumpTimestamp = now;
    }
  }
  if ((buttons&4)) {
    if (poolvalveState == open) {
      poolvalveState = closing;
      poolvalveTimestamp = now;
    }
    if (poolvalveState == closed) {
      poolvalveState = opening;
      poolvalveTimestamp = now;
    }
  }
  if ((buttons&8)) {
    if (tankvalveState == open) {
      tankvalveState = closing;
      tankvalveTimestamp = now;
    }
    if (tankvalveState == closed) {
      tankvalveState = opening;
      tankvalveTimestamp = now;
    }
  }
  */
}

void change_actuators(void)
{
  if ((runningPump == small || runningPump == both)
      && (tankvalveState == open || poolvalveState == open)) {
    digitalWrite(PIN_SMALLPUMP, HIGH);
  } else {
    digitalWrite(PIN_SMALLPUMP, LOW);
  }
  if ((runningPump == big || runningPump == both)
      && (tankvalveState == open || poolvalveState == open)) {
    digitalWrite(PIN_BIGPUMP, HIGH);
  } else {
    digitalWrite(PIN_BIGPUMP, LOW);
  }

  if (tankvalveState == opening) {
    digitalWrite(PIN_TANKVALVE_A, HIGH);
    digitalWrite(PIN_TANKVALVE_B, LOW); 
  } else if (tankvalveState == closing) {
    digitalWrite(PIN_TANKVALVE_A, LOW);
    digitalWrite(PIN_TANKVALVE_B, HIGH); 
  } else {
    digitalWrite(PIN_TANKVALVE_A, LOW);
    digitalWrite(PIN_TANKVALVE_B, LOW);
  }
  if (poolvalveState == opening) {
    digitalWrite(PIN_POOLVALVE, HIGH);
  } else if (poolvalveState == closing) {
    digitalWrite(PIN_POOLVALVE, LOW); 
  } else {
    // deixa como esta'...
  }
}

void setDisplay(void)
{
  byte t;
  

  byte n = (now/2500)%7;
  if (n>=1 && n<=5) {
    displayTemp(n);
  } else if (n==0) {
    disp.setDisplayToDecNumber(now/1000, 0, false);
  } else {
    disp.setDisplayToDecNumber(flow, 0, false);
  }
  if (int_flow_count&8) digitalWrite(PIN_LED, HIGH);
  else                  digitalWrite(PIN_LED, LOW);
  disp.setLEDs(0x0800*((tankvalveState == opening)||(tankvalveState==closing)||(tankvalveState==open))
              |0x0008*((tankvalveState == opening)||(tankvalveState==closing)||(tankvalveState==closed))
              |0x0400*((poolvalveState == opening)||(poolvalveState==closing)||(poolvalveState==open))
              |0x0004*((poolvalveState == opening)||(poolvalveState==closing)||(poolvalveState==closed))
              |0x0200*((pumpState == starting)||(runningPump == small))
              |0x0002*((runningPump == small)||(runningPump == both))
              |0x0100*((pumpState == starting)||(runningPump == big))
              |0x0001*((runningPump == big)||(runningPump == both)));
}

void loop() {
  now = millis();

  read_sensors();

  verify_comm();

  verify_status();
  change_actuators();

  setDisplay();
}
