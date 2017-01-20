void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

  float v = 0;
void loop() {
  float p;
  // put your main code here, to run repeatedly:

  v = v*0.999 + analogRead(A0) * 5 / 1023.0 * 0.001;
  p = (v - 0.5) * 0.5 / 4 * 102;
  Serial.println(p);
}
