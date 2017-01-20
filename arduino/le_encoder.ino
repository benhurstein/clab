#include <digitalWriteFast.h>

void setup() {
  // put your setup code here, to run once:
  pinModeFast(2, INPUT);
  pinModeFast(3, INPUT);
  Serial.begin(115200);
  attachInterrupt(0, cutuque, FALLING);

}

volatile int ct = 0;
volatile int max = 0;
volatile int min = 10000;
volatile int ct0 = 0;
volatile int ct1 = 0;

void cutuque()
{
  if (digitalReadFast(3) == HIGH) ct++; 
  else ct--;
  //if (ct > max) max = ct; if (ct < min) min = ct;
}

void loop() {
  if (ct<min) min=ct;
  if (ct>max) max=ct;
    Serial.print(min);
    Serial.print(" ");
    Serial.println(max);
  
}

void loopa() {
  // put your main code here, to run repeatedly:
  //Serial.println(ct);
  //return;
  //delay(500);
  while (digitalRead(2) == LOW);
  //ct += digitalRead(3);
  //Serial.println(ct);
  //if (ct > max) max = ct; if (ct < min) min = ct;
  while (digitalRead(2) == HIGH);
  if (digitalRead(3) == HIGH) ct++; else {
    ct--;
  //Serial.println(ct);
}
  if (ct > max) max = ct;
  if (ct < min) min = ct;
  if (ct == 0) {
    Serial.print(min);
    Serial.print(" ");
    Serial.println(max);
  }
}

