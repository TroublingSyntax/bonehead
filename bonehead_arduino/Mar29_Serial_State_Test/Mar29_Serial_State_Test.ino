#include "string.h"

/*
Robot state's enumeration in code.
NONE = 0
STARTING = 1
ACTIVE = 2
STOPPING = 3
*Possible future states include QUAD, WHEELS*
*/
enum state {none, starting, active, stopping} bonehead_state = none;

String input;

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    input = Serial.readString();
  }
  bonehead_state = input.toInt();
  switch (bonehead_state) {
    case starting:
      Serial.println("starting Bonehead...");
      // send legs to home position (90's)
      delay(3000);
      break;
    case active:
      Serial.println("Bonehead is active");
      // transition from 90's to stance position (?)
      delay(3000);
      break;
    case stopping:
      Serial.println("stopping Bonehead...");
      // transtion back to stance, then to 90's and wait
      delay(3000);
      break;
    default:
      Serial.println("no state detected!");
      delay(3000);
      break;
  }
}