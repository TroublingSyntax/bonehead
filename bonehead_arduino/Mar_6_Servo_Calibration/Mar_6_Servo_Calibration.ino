#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "string.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);



//-90 degrees is 163
//0 degrees is 292
//90 degrees is 420
#define SERVO_FREQ 50 //Analog servos run at ~50 Hz updates



String old_str = "";
String new_str = "";

String val;
int testpwm = 292;

bool isValid = 1;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pwm1.begin();
  pwm1.setOscillatorFrequency(27000000);
  pwm1.setPWMFreq(SERVO_FREQ);

  pwm2.begin();
  pwm2.setOscillatorFrequency(27000000);
  pwm2.setPWMFreq(SERVO_FREQ);

  delay(10);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0)
  {
  new_str = Serial.readString();
  }
  if (new_str != old_str)
  {

    parseString(new_str);

    pwm1.setPWM(3, 0, testpwm);
    Serial.println(testpwm);
  }

  else
  {
    pwm1.setPWM(3, 0, testpwm);
  }




}


void parseString(String str)
{
  val = str.substring(0, 4);
  testpwm = val.toInt();
}



