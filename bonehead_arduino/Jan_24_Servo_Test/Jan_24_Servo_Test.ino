#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "string.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//#define SERVOMIN  163 // This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  420 // This is the 'maximum' pulse length count (out of 4096)
//#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
//#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

<<<<<<< HEAD
#define L1 2.25
#define L2 4.75//4.90
#define L3 6.00
=======
#define L1 56
#define L2 108.26
#define L3 121.63
>>>>>>> 73a82dde98bdc6804887cc26afe086911de73e13

String old_str = "291.5";
String new_str = "3291.5";

String x;
String y;
String z;

float val1, val2, val3;

double th1 = 0;
double th2 = 0;
double th3 = 0;

float pwm1, pwm2, pwm3;

bool isValid = 1;

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
}

void loop() {
  if (Serial.available() > 0)
  {
  new_str = Serial.readString();
  }
  if (new_str != old_str)
  {

    parseString(new_str);
    isValid = IK(val1,val2,val3);
    isValid = set_servos();

    Serial.print("th1: ");
    Serial.println(th1);
    Serial.print("th2: ");
    Serial.println(th2);
    Serial.print("th3: ");
    Serial.println(th3);
    old_str = new_str;
  }
  else
  {
    isValid = set_servos();
  }
}

bool set_servos()
{
  if (!isValid)
  {
    return 0;
  }
  else
  {
  // map functions
  int temp_pwm1 = own_map(th1,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm1 < 163) || (temp_pwm1 > 420))
  {
    return 0;
  }
  else
  {
    pwm1 = temp_pwm1;
    pwm.setPWM(0, 0, pwm1);
  }
  int temp_pwm2 = own_map(th2,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm2 < 163) || (temp_pwm2 > 420))
  {
    return 0;
  }
  else
  {
    pwm2 = temp_pwm2;
    pwm.setPWM(1, 0, pwm2);
  }
  int temp_pwm3 = own_map(th3,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm3 < 163) || (temp_pwm3 > 420))
  {
    return 0;
  }
  else
  {
    pwm3 = temp_pwm3;
    pwm.setPWM(2, 0, pwm3);
  }

  return 1;
  }
}

bool IK(float x, float y, float z)
{
  x = x - 21; // offset to fix x direction error
  double r = sqrt(((y * y) + (z * z)));
  double this_th1 = atan2(z,y);
  double this_th2 = acos((L1/r));
  double temp_th1 = this_th2 - this_th1;
  //Check for validity
  if ((th1 < -HALF_PI) || (th1 > HALF_PI))
  {
    return 0;
  }
  else
  {
    th1 = temp_th1 * -1;
  }

  double rprime = sqrt(((x * x) + (r * r)));
  double this_th = atan2(x,r);
  double phi = acos(((L2 * L2) + (rprime * rprime) - (L3 * L3)) / (2 * L2 * rprime));
  double temp_th2 = phi - this_th;
  if ((th2 < -HALF_PI) || (th2 > HALF_PI))
  {
    return 0;
  }
  else
  {
    th2 = temp_th2;
  }
  double psi = acos(((L2 * L2) + (L3 * L3) - (rprime * rprime)) / (2 * L2 * L3));
  double temp_th3 = psi - HALF_PI;
  if ((th3 < -HALF_PI) || (th3 > HALF_PI))
  {
    return 0;
  }
  else
  {
    th3 = temp_th3;
  }
  Serial.print("th1: ");
  Serial.println(th1);
  Serial.print("th2: ");
  Serial.println(th2);
  Serial.print("th3: ");
  Serial.println(th3);
  return 1;
}

void parseString(String str)
{
  x = str.substring(0, 4);
  y = str.substring(5, 9);
  z = str.substring(10, 14);

  val1 = x.toFloat();
  val2 = y.toFloat();
  val3 = z.toFloat();
}

 int own_map(double val, double prev_low, double prev_high, double now_low, double now_high)
 {
  double prev_range = prev_high - prev_low;
  double now_range = now_high - now_low;
  double prev_location = (val - prev_low) / prev_range;
  double now_location = (prev_location * now_range) + now_low;
  return now_location;
 }
