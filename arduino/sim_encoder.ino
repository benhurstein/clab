void setup() {
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

}

#define D 12

static int j;
void loop() {
  for (int i = 0; i < 1000; i++) {
    digitalWrite(2, HIGH);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(3, HIGH);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(2, LOW);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(3, LOW);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
  }
  for (int i = 0; i < 1000; i++) {
    digitalWrite(2, HIGH);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(3, LOW);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(2, LOW);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
    digitalWrite(3, HIGH);
    for (j = 0; j < D; j++); //delayMicroseconds(D);
  }
  //delay(1000);
}
