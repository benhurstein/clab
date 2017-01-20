void setup() {
  // put your setup code here, to run once:
  pinMode(0, INPUT);
  pinMode(1, INPUT);

}

int ct = 0;
int max = 0;
int min = 1000;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(ct);
  while(digitalRead(0) == LOW);
  ct += digitalRead(1);
  Serial.println(ct);
  if(ct>max) max=ct; if(ct<min) min=ct;
  while(digitalRead(0) == HIGH);
  ct += digitalRead(1);
  Serial.println(ct);
  if(ct>max) max=ct; if(ct<min) min=ct;
  if(ct==0) {Serial.print(min); Serial.print(" ");Serial.println(max);}
}
