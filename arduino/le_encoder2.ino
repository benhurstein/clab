//#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder enc(2, 3);
long min=10000;
long max=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

}

void loop() {
  long v;
  v = enc.read();
  if (v<min) min=v;
  if (v>max) max=v;
  Serial.print(min);
  Serial.print(" ");
  Serial.println(max);
  // put your main code here, to run repeatedly:

}
