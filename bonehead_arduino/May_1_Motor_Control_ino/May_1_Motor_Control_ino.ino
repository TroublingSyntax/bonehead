#include <L298NX2.h> 

// Pin definition
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



// Initialize both motors
//L298NX2 motorsL(EN_LA, IN1_LA, IN2_LA, EN_LB, IN1_LB, IN2_LB);
//L298NX2 motorsR(EN_RA, IN1_RA, IN2_RA, EN_RB, IN1_RB, IN2_RB);

// char motorNumber = 'A'; // (A,B,C,D) 
// char motorDirection = 'F'; // (F, S, R)
// int motorSpeed = 255; // (0 -255)

void setup() {
  Serial.begin(9600);
  //Setup pin modes
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
}

void loop() {

  
 
  driveMotor(i2c.motor_dir[0], i2c.motor_cmds[0], 1);
  //setDirection(i2c.motor_dir[0], i2c.motor_dir[1], i2c.motor_dir[2], i2c.motor_dir[3]);
  //motorsL.setSpeedA(i2c.motor_cmds[0]);
  //motorsL.setSpeedB(i2c.motor_cmds[1])
  //motorsR.setSpeedA(i2c.motor_cmds[2]);
  //motorsR.setSpeedB(i2c.motor_cmds[3])


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






// void setDirection(char motor1Direction, char motor2Direction, char motor3Direction, char motor4Direction) {

//  switch(motor1Direction)
//  {
//   case 'F':
//     motorsL.forwardA();
//     break;
//   case 'S':
//     motorsL.stopA();
//     break;
//   case 'R':
//     motorsL.backwardA();
//     break;
//  }

//  switch(motor2Direction)
//  {
//   case 'F':
//     motorsL.forwardB();
//     break;
//   case 'S':
//     motorsL.stopB();
//     break;
//   case 'R':
//     motorsL.backwardB();
//     break;
//   }

// switch(motor3Direction)
// {
//   case 'F':
//     motorsR.forwardA();
//     break;
//   case 'S':
//     motorsR.stopA();
//     break;
//   case 'R':
//     motorsR.backwardA();
//     break;
// }

// switch(motor4Direction)
// {
//   case 'F':
//     motorsR.forwardB();
//     break;
//   case 'S':
//     motorsR.stopB();
//     break;
//   case 'R':
//     motorsR.backwardB();
//     break;
// }
// }
