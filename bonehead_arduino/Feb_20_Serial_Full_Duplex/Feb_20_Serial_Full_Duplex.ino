/*
Full Duplex Arduino Communication
  This sketch combines the "Feb_15_Serial_Test.ino" and "Feb_16_Serial_Write_Test.ino" files
  to perform both TX and RX with ROS via serial communication. This sketch is to be the basis
  for the final Arduino program in the bonehead project.
*/

#include "i2c.hpp"
#include "TimerInterrupt.hpp" 

// IMU variables
//const int MPU = 0x68; // MPU6050 I2C address
//float AccX, AccY, AccZ;
//float GyroX, GyroY, GyroZ;
//float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
//float roll, pitch, yaw;
//float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
//float elapsedTime, currentTime, previousTime;
//int c = 0;

//****************GLOBAL SCOPE SERVO STUFF*******************************************
#if USING_ACTUATORS
// Servo members
Adafruit_PWMServoDriver PCA_1 = Adafruit_PWMServoDriver(0x40); // unbridged servo driver
Adafruit_PWMServoDriver PCA_2 = Adafruit_PWMServoDriver(0x41); // bridged servo driver
const int SERVO_FREQ = 50;
#endif


// Pin definition of dc motors
const unsigned int EN_RA = 18;  //M1
const unsigned int IN1_RA = 0;
const unsigned int IN2_RA = 15;

const unsigned int IN1_RB = 1;  //M2
const unsigned int IN2_RB = 16;
const unsigned int EN_RB = 19;

const unsigned int EN_LA = 3; //M3
const unsigned int IN1_LA = 21;
const unsigned int IN2_LA = 5;

const unsigned int IN1_LB = 20;  //M4
const unsigned int IN2_LB = 4;
const unsigned int EN_LB = 2;

int servo_cmds[12] = {292}; // holds the servo commands from ROS



String inputBuffer; // String to hold raw input data
char thisByte; // character to hold the immediate input byte
// internal command arrays for the actuators
//uint8_t motor_cmds[4] = {0,0,0,0};
//uint8_t motor_dir[4] = {3,3,3,3};
//uint16_t servo_cmds[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

I2C i2c;

void setup() {
  Serial.begin(115200);
    while (!Serial) {
    ; // Wait until the serial port is open
  }

  pinMode(LED_BUILTIN, OUTPUT);

  //SETUP MOTOR PINS
  pinMode(EN_RA, OUTPUT);
  pinMode(IN1_RA, OUTPUT);
  pinMode(IN2_RA, OUTPUT);

  pinMode(EN_RB, OUTPUT);
  pinMode(IN1_RB, OUTPUT);
  pinMode(IN2_RB, OUTPUT);

  pinMode(EN_LA, OUTPUT);
  pinMode(IN1_LA, OUTPUT);
  pinMode(IN2_LA, OUTPUT);

  pinMode(EN_LB, OUTPUT);
  pinMode(IN1_LB, OUTPUT);
  pinMode(IN2_LB, OUTPUT);

  delay(100);

  Serial.print(F("\nStarting Argument_None on ")); Serial.println(BOARD_NAME);
  Serial.println(SAMD_TIMER_INTERRUPT_VERSION);
  Serial.print(F("CPU Frequency = ")); Serial.print(F_CPU / 1000000); Serial.println(F(" MHz"));
  
  // Interval in millisecs
  
  if (ITimer.attachInterruptInterval_MS(TIMER_INTERVAL_MS, set_servos))
  {
    preMillisTimer = millis();
    Serial.print(F("Starting  ITimer OK, millis() = ")); Serial.println(preMillisTimer);
  }
  else { Serial.println(F("Can't set ITimer. Select another freq. or timer")); }

  #if USING_ACTUATORS
  PCA_1.begin();
  PCA_1.setOscillatorFrequency(27000000);
  PCA_1.setPWMFreq(SERVO_FREQ);
  
  PCA_2.begin();
  PCA_2.setOscillatorFrequency(27000000);
  PCA_2.setPWMFreq(SERVO_FREQ);
  #endif

  i2c.initI2C();
}

void loop() {
  //i2c.get_orientation(); // get the euler angles
  //i2c.get_GPS(); // get the latitude and longitude
  //sendFloat(i2c.euler_angles[0], 'a'); // send the roll
  //sendFloat(i2c.euler_angles[1], 'b'); // send the pitch
  //sendFloat(i2c.euler_angles[2], 'c'); // send yaw
  //sendFloat(i2c.heading, 'd'); // send the heading
  //sendFloat(i2c.latitude, 'e'); // send the latitude
  //sendFloat(i2c.longitude, 'f'); // send the longitude
  CheckSerial(); // check the serial bus for new data
  //i2c.set_servos();
  //Serial.print("Heading: ");
  //Serial.print(i2c.heading);
  //Serial.print("   Roll: ");
  //Serial.print(i2c.euler_angles[0]);
  //Serial.print("   Pitch: ");
  //Serial.print(i2c.euler_angles[1]);
  //Serial.print("   Yaw: ");
  //Serial.print(i2c.euler_angles[2]);I don't understand what you are trying to achieve. The Arduino HardwareSerial code handles the actual USART interrupts so you don't need to.
  //Serial.print("   Latitude: ");
  //Serial.print(i2c.latitude);
  //Serial.print("   Longitude: ");
  //Serial.println(i2c.longitude);
  
  //DC motor commands
  driveMotor(i2c.motor_dir[0], i2c.motor_cmds[0], 1);
  driveMotor(i2c.motor_dir[1], i2c.motor_cmds[1], 2);
  driveMotor(i2c.motor_dir[2], i2c.motor_cmds[2], 3);
  driveMotor(i2c.motor_dir[3], i2c.motor_cmds[3], 4);
}

void sendFloat(float f, char header) {
  byte * b = (byte *) &f;
  Serial.write(header);
  Serial.write(b, 4);
  Serial.print('z');
  Serial.flush();
  return;
}

void CheckSerial() {
  /*
  A function to handle data being in
  the UART input buffer from anywhere
  in the program loop.
  */
  if (Serial.available() > 0)
  {
  char thisByte = Serial.read(); // reads a single byte from the UART input buffer
  inputBuffer += thisByte; // append the newest byte to the programs input buffer
  if (inputBuffer.length() > 20)
  {
    inputBuffer = "";
    inputBuffer.trim();
  }
  if (thisByte == 'z') { // if the input buffer is a complete message, parse it
    InputParse();
  }
  }
  else {
    return;
  }
}

void InputParse()
{
  /*
  A function that parses the input buffer
  and places the input command into the
  appropriate variable.
  */
  inputBuffer.trim();
  char header = inputBuffer.charAt(0); // get the input message header
  inputBuffer.remove(0,1); // remove the header after reading it
  inputBuffer.remove(inputBuffer.length(),1); // remove the end character
  int input_val = inputBuffer.toInt(); // convert to double
  switch (header)
  {
    // motor commands parsed here
    case 'a':
      i2c.motor_cmds[0] = input_val;
      break;
    case 'b':
      i2c.motor_dir[0] = input_val;
      break;
    case 'c':
      i2c.motor_cmds[1] = input_val;
      break;
    case 'd':
      i2c.motor_dir[1] = input_val;
      break;
    case 'e':
      i2c.motor_cmds[2] = input_val;
      break;
    case 'f':
      i2c.motor_dir[2] = input_val;
      break;
    case 'g':
      i2c.motor_cmds[3] = input_vcase 1al;
      break;
    case 'h':
      i2c.motor_dir[3] = input_val;
      break;
    // servo commands parsed here   
    case 'i':
      servo_cmds[0] = input_val;
      break;
    case 'j':
      servo_cmds[1] = input_val;
      break;
    case 'k':
      servo_cmds[2] = input_val;
      break;
    case 'l':
      servo_cmds[3] = input_val;
      break;
    case 'm':
      servo_cmds[4] = input_val;
      break;
    case 'n':
      servo_cmds[5] = input_val;
      break;
    case 'o':
      servo_cmds[6] = input_val;
      break;
    case 'p':
      servo_cmds[7] = input_val;
      break;
    case 'q':
      servo_cmds[8] = input_val;
      break;
    case 'r':
      servo_cmds[9] = input_val;
      break;
    case 's':
      servo_cmds[10] = input_val;
      break;
    case 't':
      servo_cmds[11] = input_val;
      break;

    default:
      break;
  }
  inputBuffer = ""; // clear the programs input buffer after completed message
  return;
}

  void set_servos()
  {
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
    
    return;
  }


void driveMotor(int motorDirection, int motorSpeed, int motorNum)
{
  
    if (motorSpeed == 0)
    {
      motorDirection = 2;
    }
  switch (motorNum)
  {
    case 1:
      switch(motorDirection)
      {
        case 0:
          digitalWrite(IN1_RA, HIGH);
          digitalWrite(IN2_RA, LOW);
          analogWrite(EN_RA, motorSpeed);
        break;
        
        case 1:
        digitalWrite(IN1_RA, LOW);
        digitalWrite(IN2_RA, HIGH);
        analogWrite(EN_RA, motorSpeed);
        break;

        case 2:
        digitalWrite(IN1_RA, LOW);
        digitalWrite(IN2_RA, LOW);
        analogWrite(EN_RA, motorSpeed);
        break;
      }
      break;  


  case 2:
      switch(motorDirection)
      {
        case 0:
          digitalWrite(IN1_RB, HIGH);
          digitalWrite(IN2_RB, LOW);
          analogWrite(EN_RB, motorSpeed);
        break;
        
        case 1:
        digitalWrite(IN1_RB, LOW);
        digitalWrite(IN2_RB, HIGH);
        analogWrite(EN_RB, motorSpeed);
        break;

        case 2:
        digitalWrite(IN1_RB, LOW);
        digitalWrite(IN2_RB, LOW);
        analogWrite(EN_RB, motorSpeed);
        break;
      }
      break;  


  case 3:
      switch(motorDirection)
      {
        case 0:
          digitalWrite(IN1_LA, HIGH);
          digitalWrite(IN2_LA, LOW);
          analogWrite(EN_LA, motorSpeed);
        break;
        
        case 1:
        digitalWrite(IN1_LA, LOW);
        digitalWrite(IN2_LA, HIGH);
        analogWrite(EN_LA, motorSpeed);
        break;

        case 2:
        digitalWrite(IN1_LA, LOW);
        digitalWrite(IN2_LA, LOW);
        analogWrite(EN_LA, motorSpeed);
        break;
      }
      break;


  case 4:
      switch(motorDirection)
      {
        case 0:
          digitalWrite(IN1_LB, HIGH);
          digitalWrite(IN2_LB, LOW);
          analogWrite(EN_LB, motorSpeed);
        break;
        
        case 1:
        digitalWrite(IN1_LB, LOW);
        digitalWrite(IN2_LB, HIGH);
        analogWrite(EN_LB, motorSpeed);
        break;

        case 2:
        digitalWrite(IN1_LB, LOW);
        digitalWrite(IN2_LB, LOW);
        analogWrite(EN_LB, motorSpeed);
        break;
      }
      break;
  }  

}


