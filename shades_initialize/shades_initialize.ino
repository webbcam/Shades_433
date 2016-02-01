/*
 * Initializes Chip's EEPROM for first use.
 */

#include <EEPROM.h>

#define LED A1

void setup() {
  pinMode(LED, OUTPUT);
  // put your setup code here, to run once:
  EEPROM.write(0, 0);   //  set shade_ID to 0
  EEPROM.write(1, 2);   //  point remote_ID to first open space
  //  clear out remote_ID
  for (int i = 2; i < 8; i++) {
    EEPROM.write(i, 0);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, LOW);

}
