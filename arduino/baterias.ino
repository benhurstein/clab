#include <OneWire.h>
#include <TM1638.h>

//
// Controle das baterias dos ventiladores 4x12 = 48V
//
// Faz o seguinte:
// - controla a carga das baterias que formam os 48V necessários
//   aos ventiladores (2 de recirculação, 1 de entrada de ar novo
//   e 1 de saída de ar viciado)
// Para isso, le dados de:
// - por enquanto, nada
// Funcionamento por enquanto:
// - os 48V são formados por 4 baterias de 12V em série.
//   O carregador funciona em 12 ou em 24V. Está funcionando
//   em 24V, carregando 2 baterias por vez. Este programa está
//   selecionando quais dos conjuntos de 2 baterias sao
//   recarregados por vez. Deixa um tempo em cada conjunto, que sao
//   ligados por relés. O programa é o blink em baixa frequencia.
// Como deve ser:
// - deve controlar a carga individual de cada bateria, e carregar
//   ou nao de acordo com a necessidade.
// - atualmente a energia vem de 220V -> 12V -> 26V, mais tarde deve
//   ter a opcao de vir de paineis solares
// - pode ser tambem o controlador dos ventiladores e dos registros
//   de recirculacao do ar. Ou nao, deixar exclusivamente como 
//   controlador das baterias, talvez o controlador do solar mais 
//   tarde.
//

#define SECONDS(s) ((s)*1000L)
#define MINUTES(m) ((m)*SECONDS(60))

#define CHARGINGTIME MINUTES(30)

// Estado do sistema
//{

  enum { bank1, bank2 } chargingBatteries;
  long chargingBatteriesTimestamp;

  long now;
//}


// pinos

#define PIN_BANK_SELECT 13

TM1638 disp(A2, A1, A3, true, 1);


void setup() {
  Serial.begin(115200);

  pinMode(PIN_BANK_SELECT, OUTPUT);
  digitalWrite(PIN_BANK_SELECT, HIGH);

  chargingBatteries = bank1;
  chargingBatteriesTimestamp = -CHARGINGTIME/2;

  disp.setDisplayToString("Start");
  
  delay(1000);
}

void display_status(void)
{
  char s[9];

  int dt = (CHARGINGTIME - (now - chargingBatteriesTimestamp))
         / SECONDS(1);

  sprintf(s, "%s%6d", chargingBatteries == bank1 ? "B1" : "B2", dt);
  disp.setDisplayToString(s);
  //Serial.println(s);
}

void read_sensors(void)
{
  // 
}

void verify_status(void)
{

  if (chargingBatteries == bank1) {
    unsigned long dt = now - chargingBatteriesTimestamp;
    if (dt >= CHARGINGTIME) {
      chargingBatteries = bank2;
      chargingBatteriesTimestamp = now;
    }
  }
  if (chargingBatteries == bank2) {
    unsigned long dt = now - chargingBatteriesTimestamp;
    if (dt >= CHARGINGTIME) {
      chargingBatteries = bank1;
      chargingBatteriesTimestamp = now;
    }
  }
}

void change_actuators(void)
{
  if (chargingBatteries == bank1) {
    digitalWrite(PIN_BANK_SELECT, HIGH);
  }
  if (chargingBatteries == bank2) {
    digitalWrite(PIN_BANK_SELECT, LOW);
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
