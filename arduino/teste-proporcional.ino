/*
 Fade

 This example shows how to fade an LED on pin 9
 using the analogWrite() function.

 The analogWrite() function uses PWM, so if
 you want to change the pin you're using, be
 sure to use another PWM capable pin. On most
 Arduino, the PWM pins are identified with 
 a "~" sign, like ~3, ~5, ~6, ~9, ~10 and ~11.

 This example code is in the public domain.
 */

int led = 9;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 10;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
  Serial.begin(115200);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin 9:

  //int i;
  brightness=255;
  //for (i=0; i<3; i++){
  analogWrite(led, brightness);
  //digitalWrite(13, HIGH);
  //delayMicroseconds(30/*(brightness)*10*/);
  //digitalWrite(13, LOW);
  //delayMicroseconds((255-brightness)*60);
  //return;
 // }

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(1000);
//    if ((now-tankPumpTimestamp)<1000) return;
//    tankPumpTimestamp = now;
  static uint8_t tank_smallpump_value;
    static float e_a = 0;
    static float e_i = 0;
    static int i=0;
      static float taq = 37;
      taq = taq * 0.99 + 0.01 *(25.0+0.1*tank_smallpump_value);
      float e = 41.0 - taq;
      e_i += e;
      float e_d = e - e_a;
      float kp = 30, ki = 0.01, kd = 0.0;
      float p = kp * e + ki * e_i + kd * e_d;
      e_a = e;

      //float m = map(p, 0.0, 1.0, 0.0, 255.0);
      //tank_smallpump_value = constrain(m, 0, 255);
      if (p<0) tank_smallpump_value = 0;
      else if (p>1) tank_smallpump_value = 255;
      else tank_smallpump_value = 255.0 * p;
      Serial.print(taq);
      Serial.print(" ");
      Serial.print(kp*e);
      Serial.print(" ");
      Serial.print(ki*e_i);
      Serial.print(" ");
      Serial.print(kd*e_d);
      Serial.print(" ");
      Serial.print(p);
      Serial.print(" ");
      Serial.println(tank_smallpump_value);

}
