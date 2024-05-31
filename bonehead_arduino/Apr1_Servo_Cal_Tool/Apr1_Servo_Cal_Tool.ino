#include <Adafruit_PWMServoDriver.h>
#include "string.h"

int fr1 = 292;
int fr2 = 292;
int fr3 = 292;

int fl1 = 292;
int fl2 = 292;
int fl3 = 292;

int br1 = 292;
int br2 = 292;
int br3 = 292;

int bl1 = 292;
int bl2 = 292;
int bl3 = 292;

int servo_cmds[12] = {292};

Adafruit_PWMServoDriver PCA_1 = Adafruit_PWMServoDriver(0x40); // unbridged servo driver
Adafruit_PWMServoDriver PCA_2 = Adafruit_PWMServoDriver(0x41); // bridged servo driver
const int SERVO_FREQ = 50;

String input;

void setup() {
  Serial.begin(19200);

  PCA_1.begin();
  PCA_1.setOscillatorFrequency(27000000);
  PCA_1.setPWMFreq(SERVO_FREQ);
  
  PCA_2.begin();
  PCA_2.setOscillatorFrequency(27000000);
  PCA_2.setPWMFreq(SERVO_FREQ);
}

void loop() {
  if (Serial.available()) {
    input = Serial.readString();
    input.trim();
    if (input.charAt(input.length()-1) == 'z') {
     parseString(); 
    }
  }
  // front right
  PCA_2.setPWM(3, 0, servo_cmds[0]);
  PCA_2.setPWM(4, 0, servo_cmds[1]);
  PCA_2.setPWM(5, 0, servo_cmds[2]);

  // front left
  PCA_2.setPWM(0, 0, servo_cmds[3]);
  PCA_2.setPWM(1, 0, servo_cmds[4]);
  PCA_2.setPWM(2, 0, servo_cmds[5]);

  // back right
  PCA_1.setPWM(3, 0, servo_cmds[6]);
  PCA_1.setPWM(4, 0, servo_cmds[7]);
  PCA_1.setPWM(5, 0, servo_cmds[8]);

  // back left
  PCA_1.setPWM(0, 0, servo_cmds[9]);
  PCA_1.setPWM(1, 0, servo_cmds[10]);
  PCA_1.setPWM(2, 0, servo_cmds[11]);
}

void parseString() {
  char header = input.charAt(0);
  Serial.println(input);
  input.remove(0,1); // remove the header from the string
  input.remove(input.length()-1); // remove the end character
  switch (header) {
    case 'a':
      servo_cmds[0] = input.toInt();
      break;
    case 'b':
      servo_cmds[1] = input.toInt();
      break;
    case 'c':
      servo_cmds[2] = input.toInt();
      Serial.println("got here");
      break;
    case 'd':
      servo_cmds[3] = input.toInt();
      break;
    case 'e':
      servo_cmds[4] = input.toInt();
      break;
    case 'f':
      servo_cmds[5] = input.toInt();
      break;
    case 'g':
      servo_cmds[6] = input.toInt();
      break;
    case 'h':
      servo_cmds[7] = input.toInt();
      break;
    case 'i':
      servo_cmds[8] = input.toInt();
      break;
    case 'j':
      servo_cmds[9] = input.toInt();
      break;
    case 'k':
      servo_cmds[10] = input.toInt();
      break;
    case 'l':
      servo_cmds[11] = input.toInt();
      break;
    default:
      break;
  }
  return;
}