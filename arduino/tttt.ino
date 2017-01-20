/*
Library examples for TM1638.

Copyright (C) 2011 Ricardo Batista <rjbatista at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <TM1638.h>

// define a module on data pin 8, clock pin 9 and strobe pin 7
TM1638 module(5, 6, 7);

void setup() {
  Serial.begin(115200);
  // display a hexadecimal number and set the left 4 dots
  module.setDisplayToHexNumber(0x1234ABCD, 0xF0);
  module.setDisplayToString("l a r a ");

  int i;
  for (i=0; i<=A7;) {
  Serial.print(i);
  Serial.print(" ");
  Serial.print(digitalPinToPort(i));
  Serial.println(digitalPinToBitMask(i));
  if(i==13) i=A0; else i++;
    
  }
}

void loop() {
  byte keys = module.getButtons();
  //Serial.println(keys);

  // light the first 4 red LEDs and the last 4 green LEDs as the buttons are pressed
  module.setLEDs(((keys & 0xF0) << 8) | (keys & 0xF));
}
