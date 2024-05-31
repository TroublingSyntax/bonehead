#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "string.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver();
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41);

//#define SERVOMIN  163 // This is the 'minimum' pulse length count (out of 4096)
//#define SERVOMAX  420 // This is the 'maximum' pulse length count (out of 4096)
//#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
//#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates



#define L1 .055
#define L2 .139
#define L3 .148


String old_str = "";
String new_str = "";

String x;
String y;
String z;

float val1, val2, val3;

double th1 = 0;
double th2 = 0;
double th3 = 0;

float sig1L, sig2L, sig3L, sig1R, sig2R, sig3R;

bool isValid = 1;

void setup() {
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
  if (Serial.available() > 0)
  {
  new_str = Serial.readString();
  Serial.println(new_str);
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
  int temp_pwm1L = own_map(th1,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm1L < 163) || (temp_pwm1L > 420))
  {
    return 0;
  }
  else
  {
    sig1L = temp_pwm1L;
    pwm1.setPWM(3, 0, sig1L);
    pwm2.setPWM(0, 0, sig1L);
  }
  int temp_pwm1R = own_map(th1,-HALF_PI,HALF_PI,420,163);
  if ((temp_pwm1R < 163) || (temp_pwm1R > 420))
  {
    return 0;
  }
  else
  {
    sig1R = temp_pwm1R;
    pwm1.setPWM(0, 0, sig1R);
    pwm2.setPWM(3, 0, sig1R);
  }
  int temp_pwm2L = own_map(th2,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm2L < 163) || (temp_pwm2L > 420))
  {
    return 0;
  }
  else
  {
    sig2L = temp_pwm2L;
    pwm1.setPWM(1, 0, sig2L);
    pwm2.setPWM(1, 0, sig2L);
  }
  int temp_pwm2R = own_map(th2,-HALF_PI,HALF_PI,420,163);
  if ((temp_pwm2R < 163) || (temp_pwm2R > 420))
  {
    return 0;
  }
  else
  {
    sig2R = temp_pwm2R;
    pwm1.setPWM(4, 0, sig2R);
    pwm2.setPWM(4, 0, sig2R);
  }
  int temp_pwm3L = own_map(th3,-HALF_PI,HALF_PI,163,420);
  if ((temp_pwm3L < 163) || (temp_pwm3L > 420))
  {
    return 0;
  }
  else
  {
    sig3L = temp_pwm3L;
    pwm1.setPWM(2, 0, sig3L);
    pwm2.setPWM(2, 0, sig3L);
  }
    int temp_pwm3R = own_map(th3,-HALF_PI,HALF_PI,420,163);
  if ((temp_pwm3R < 163) || (temp_pwm3R > 420))
  {
    return 0;
  }
  else
  {
    sig3R = temp_pwm3R;
    pwm1.setPWM(5, 0, sig3R);
    pwm2.setPWM(5, 0, sig3R);
  }

  return 1;
  }
}

bool IK(float x, float y, float z)
{
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
