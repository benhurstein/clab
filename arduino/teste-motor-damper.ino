void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    pinMode(3, OUTPUT);
    digitalWrite(3, HIGH);

}

int s0 = 0;
int f = 0;
long t0 = 0;
char v[][8] = {{0,0,0,0,0,0,0,0},
               {0,0,0,0,0,0,0,1},
               {0,0,0,1,0,0,0,1},
               {0,0,1,0,1,0,0,1},
               {0,0,1,1,0,0,1,1},
               {0,1,1,0,1,0,1,1},
               {0,1,1,1,0,1,1,1},
               {0,1,1,1,1,1,1,1},
               {1,1,1,1,1,1,1,1}};
int m0;

void loop() {
  long t = millis();
  int s = t / 1000 / 5 % (sizeof(v)/sizeof(v[0]));
  int m = (t * 120 / 1000) % sizeof(v[s]);
  int c = (t * 60 / 1000) % 60;

analogWrite(3, 30+100*((t/1000)%2));
return;

if ((t/1000)%2 == 0) digitalWrite(3,LOW);
else digitalWrite(3, HIGH);
return;         

if (m == m0) return;
m0 = m;
digitalWrite(3, v[s][m]);
/*
return;

  if (s == s0) return;

  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  if (s%2 == 1) return;
  //if (f == 2) return;
  //f++;

  if (s/2%2 != m) return;
  digitalWrite(3, LOW);
  s0 = s;
*/
  Serial.print(t-t0);
  Serial.print(" ");
  Serial.print(s);
  Serial.print(" ");
  Serial.print(c);
  Serial.print(" ");
  Serial.println(m);
  t0 = t;


}
