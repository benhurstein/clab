
int pin[6]   = { A0, 2, A2, A3, A4, A5 };
int lim[6]   = { 55, 0x200, 0x200, 0x200, 0x200, 0x200 };
int s[6];
long t[6];
float p[6];
char m[8]=" ";
char *mp;
float dd[6]={1, 1.72, 1.04, 1.014, 2.29, 1};
float df[6]={0, -100, 72,   30,    -450, 0};

volatile unsigned int conta = 0;
void isr(void)
{
  conta++;
}
unsigned int gconta=0;

void setup() {
  // put your setup code here, to run once:
  int i;

  attachInterrupt(0, isr, CHANGE);
  Serial.begin(115200);

  for (i = 0; i < 6; i++) {
    pinMode(pin[i], INPUT);
    digitalWrite(pin[i], LOW);
  }
}

long aa;
void loop() {
  // put your main code here, to run repeatedly:
  int i;
  long a = micros();
  int r;
  int f = 0;
    unsigned econta=conta;
  mp = m+1;
  for (i = 0; i < 6; i++) {
    if (i == 0) {
      r = analogRead(pin[i]);
      //Serial.print(r); Serial.print(" ");
      if (abs(r - s[i]) < 4) r = s[i];
    } else r = digitalRead(pin[i]);//
    if (r != s[i]) {
//    if (i==1)Serial.println(a-t[i]);
      s[i] = r;
      p[i] = p[i]+1;//(p[i] + (a - t[i])) / 2;
      t[i] = a;
      *mp++='A'+i;
      f = 1;
    }
  }
  if (a-aa > 1000000) {
    for (i=0; i<6; i++) {
//      Serial.print(1000000/(p[i]/dd[i]+df[i]));
      Serial.print((float)p[i]/*/(a-aa)*1e6*/);
      p[i] = 0;
      Serial.print("\t");
    }
    Serial.print((float)(econta-gconta)/*/(a-aa)*1e6*/);
    gconta=econta;
    aa = a;
    Serial.print("\n");
  }
  if (f*0) {
    *mp = 0;
    Serial.print(a);
    Serial.println(m);
    //aa = a;
    /*
    Serial.print(a - aa);
    Serial.print("\t");
    for (i = 0; i < 6; i++) {
      Serial.print(s[i]);
      Serial.print(" ");
      Serial.print(p[i]);
      Serial.print("\t");
    }
    Serial.println();
    */
  }
  //aa = a;
}
