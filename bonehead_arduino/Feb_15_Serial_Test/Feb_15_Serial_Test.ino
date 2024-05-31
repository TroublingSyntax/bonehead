String inputBuffer; // String to hold raw input data
char thisByte; // character to hold the immediate input byte
// internal command arrays for the actuators
uint8_t motor_cmds[4] = {0,0,0,0};
uint8_t motor_dir[4] = {3,3,3,3};
uint16_t servo_cmds[12] = {0,0,0,0,0,0,0,0,0,0,0,0};

bool complete_message = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait until the serial port is open
  }
}

void loop() {
  CheckSerial();
  complete_message = CheckIfMessageDone();
  if (complete_message)
  {
    PrintVals();
  }
}

void CheckSerial() {
  /*
  A function to handle data being in
  the UART input buffer from anywhere
  in the program loop.
  */
  if (Serial.available() > 0) {
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
  return;
  }
  else // If there's no serial data, just return to main loop
  {
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
      motor_cmds[0] = input_val;
      break;
    case 'b':
      motor_dir[0] = input_val;
      break;
    case 'c':
      motor_cmds[1] = input_val;
      break;
    case 'd':
      motor_dir[1] = input_val;
      break;
    case 'e':
      motor_cmds[2] = input_val;
      break;
    case 'f':
      motor_dir[2] = input_val;
      break;
    case 'g':
      motor_cmds[3] = input_val;
      break;
    case 'h':
      motor_dir[3] = input_val;
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

void PrintVals()
{ //Check the values once they are done being updated.
  Serial.print("M1: ");
  Serial.print(motor_cmds[0]);
  Serial.print(" M2: ");
  Serial.print(motor_cmds[1]);
  Serial.print(" M3: ");
  Serial.print(motor_cmds[2]);
  Serial.print(" M4: ");
  Serial.println(motor_cmds[3]);

  Serial.print("D1: ");
  Serial.print(motor_dir[0]);
  Serial.print(" D2: ");
  Serial.print(motor_dir[1]);
  Serial.print(" D3: ");
  Serial.print(motor_dir[2]);
  Serial.print(" D4: ");
  Serial.println(motor_dir[3]);

  Serial.print("S1: ");
  Serial.print(servo_cmds[0]);
  Serial.print(" S2: ");
  Serial.print(servo_cmds[1]);
  Serial.print(" S3: ");
  Serial.print(servo_cmds[2]);
  Serial.print(" S4: ");
  Serial.print(servo_cmds[3]);
  Serial.print(" S5: ");
  Serial.print(servo_cmds[4]);
  Serial.print(" S6: ");
  Serial.print(servo_cmds[5]);
  Serial.print(" S7: ");
  Serial.print(servo_cmds[6]);
  Serial.print(" S8: ");
  Serial.print(servo_cmds[7]);
  Serial.print(" S9: ");
  Serial.print(servo_cmds[8]);
  Serial.print(" S10: ");
  Serial.print(servo_cmds[9]);
  Serial.print(" S11: ");
  Serial.print(servo_cmds[10]);
  Serial.print(" S12: ");
  Serial.println(servo_cmds[11]);

  for (int i = 0; i < 4; i++)
  {
    motor_cmds[i] = 0;
  }
  for (int i = 0; i < 4; i++)
  {
    motor_dir[i] = 0;
  }
  for (int i = 0; i < 12; i++)
  {
    servo_cmds[i] = 0;
  }

  return;
}

int CheckIfMessageDone()
{ // Only return 1 if all 16 actuator commands are received
  for (int i = 0; i < 4; i++)
  {
    if (motor_cmds[i] == 0)
    {
      return 0;
    }
  }
  for (int i = 0; i < 4; i++)
  {
    if (motor_dir[i] == 3)
    {
      return 0;
    }
  }
  for (int i = 0; i < 12; i++)
  {
    if (servo_cmds[i] == 0)
    {
      return 0;
    }
  }
  return 1;
}