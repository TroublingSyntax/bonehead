# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import serial
import re
import sys
import struct

from rclpy.node import Node
from serial.tools import list_ports
from bonehead_msgs.msg import MotorCmd
from bonehead_msgs.msg import ServoCmd
from std_msgs.msg import String

class Serial(Node):
    def __init__(self):
        # data members

        ###### motor data members ######
        self.prev_motor_cmd = MotorCmd()
        self.new_motor_cmd = False

        ###### servo data members ######
        self.prev_servo_cmd = ServoCmd()
        self.new_servo_cmd = False

        ###### imu data members   ######
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        ###### magnetometer data members ######
        self.heading = 0

        ###### GPS data members ######
        self.temp_latitude = 0
        self.temp_longitude = 0
        self.latitude = 0
        self.longitude = 0

        ###### serial data members ######
        self.output_buffer = []
        self.input_buffer = []
        self.input_data = 0
        self.input_header = ''
        self.input_data_bytes = [0,0,0,0]

        super().__init__('serial_comms')
        self.connect_arduino()

        self.motor_cmd_sub = self.create_subscription(MotorCmd, 'motor_cmd', self.motor_callback, 10) # subscribe to motor command topic
        self.servo_cmd_sub = self.create_subscription(ServoCmd, 'servo_cmd', self.servo_callback, 10) # subscribe to servo command topic

        self.serial_write_period = 0.05 # serial write period
        self.serial_write_timer = self.create_timer(self.serial_write_period, self.serial_write) # try to write data at ~60 Hz

        self.test_serial_pub = self.create_publisher(String, 'raw_serial_input', 10) # temporarily create a publisher to show serial confirmation

        self.serial_read_period = 0.001 # serial read period
        self.serial_read_timer = self.create_timer(self.serial_read_period, self.serial_read)
        


    def motor_callback(self, curr_motor_cmd):
        i = 0
        for i in range(4):
            if self.prev_motor_cmd.pwm[i] != curr_motor_cmd.pwm[i]:
                self.new_motor_cmd = True
            if self.prev_motor_cmd.dir[i] != curr_motor_cmd.dir[i]:
                self.new_motor_cmd = True
        if self.new_motor_cmd: # add to output buffer
            
            self.prev_motor_cmd = curr_motor_cmd

            this_cmd = str(self.prev_motor_cmd.pwm[0])
            this_dir = str(self.prev_motor_cmd.dir[0])
            self.output_buffer.append("a" + this_cmd + "z")
            self.output_buffer.append("b" + this_dir + "z")

            this_cmd = str(self.prev_motor_cmd.pwm[1])
            this_dir = str(self.prev_motor_cmd.dir[1])
            self.output_buffer.append("c" + this_cmd + "z")
            self.output_buffer.append("d" + this_dir + "z")

            this_cmd = str(self.prev_motor_cmd.pwm[2])
            this_dir = str(self.prev_motor_cmd.dir[2])
            self.output_buffer.append("e" + this_cmd + "z")
            self.output_buffer.append("f" + this_dir + "z")

            this_cmd = str(self.prev_motor_cmd.pwm[3])
            this_dir = str(self.prev_motor_cmd.dir[3])
            self.output_buffer.append("g" + this_cmd + "z")
            self.output_buffer.append("h" + this_dir + "z")
            
        else:
            return
        
    def servo_callback(self, curr_servo_cmd):
        i = 0
        for i in range(12):
            if self.prev_servo_cmd.pwm[i] != curr_servo_cmd.pwm[i]:
                self.new_servo_cmd = True
        if self.new_servo_cmd: # add to output buffer
            
            self.prev_servo_cmd = curr_servo_cmd

            this_cmd = str(self.prev_servo_cmd.pwm[0])
            self.output_buffer.append("i" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[1])
            self.output_buffer.append("j" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[2])
            self.output_buffer.append("k" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[3])
            self.output_buffer.append("l" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[4])
            self.output_buffer.append("m" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[5])
            self.output_buffer.append("n" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[6])
            self.output_buffer.append("o" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[7])
            self.output_buffer.append("p" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[8])
            self.output_buffer.append("q" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[9])
            self.output_buffer.append("r" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[10])
            self.output_buffer.append("s" + this_cmd + "z")

            this_cmd = str(self.prev_servo_cmd.pwm[11])
            self.output_buffer.append("t" + this_cmd + "z")
            
        else:
            return
 
    def serial_write(self):
        try:
            if self.new_motor_cmd or self.new_servo_cmd: # only write commands if they are new
                i = 0
                for i in range(len(self.output_buffer)):
                    self.serial_port.write(self.output_buffer[i].encode())
                    i += 1
                self.output_buffer = []
                self.new_motor_cmd = False
                self.new_servo_cmd = False
                return
            else:
                return
        except OSError:
            self.connect_arduino()

    def serial_read(self):
        try:
            if self.serial_port.in_waiting == 0:
                return
            else:
                this_byte = self.serial_port.read(1) # read in one byte
                try:
                    this_char = this_byte.decode()
                    if this_char == 'z': # if it's the end of the data, then parse the input buffer
                        self.parseInput()
                    else:
                        self.input_buffer.append(this_byte) # if it's not the end, append the byte to the input buffer
                        return
                except UnicodeDecodeError:
                    self.input_buffer.append(this_byte) # if it's not the end, append the byte to the input buffer
        except OSError:
            self.connect_arduino()
    def parseInput(self):
        try:
            self.input_header = self.input_buffer.pop(0) # gets the data header
            # reads the four bytes of data and puts into array of bytes
            self.input_data_bytes[0] = self.input_buffer.pop(0)
            self.input_data_bytes[1] = self.input_buffer.pop(0)
            self.input_data_bytes[2] = self.input_buffer.pop(0)
            self.input_data_bytes[3] = self.input_buffer.pop(0)
        except IndexError:
            return

        self.input_buffer.clear() # clears the buffer

        # unpack the bytes list into a floating point number
        input_string = b''.join(self.input_data_bytes)
        self.input_data = struct.unpack('<f', input_string)[0]

        match self.input_header.decode(): # decide where to put the input data based on the header
            case 'a':
                self.roll = self.input_data
            case 'b':
                self.pitch = self.input_data
            case 'c':
                self.yaw = self.input_data
            case 'd':
                self.heading = self.input_data
            case 'e':
                self.temp_latitude = self.input_data
                if self.temp_latitude != 0:
                    self.latitude = self.temp_latitude
            case 'f':
                self.temp_longitude = self.input_data
                if self.temp_longitude != 0:
                    self.longitude = self.temp_longitude
            case _:
                return
        self.input_data = 0 # reset input data back to zero
        self.input_data_bytes = [0,0,0,0] # reset input data bytes back to zero
        return

    def connect_arduino(self):
        self.arduino_port = None
        while (self.arduino_port == None): # retrieve Arduino device, if any
            self.arduino_port = get_arduino_ports()

        self.serial_port = serial.Serial(self.arduino_port.device, 115200, timeout=1) # initialize serial communication
        self.serial_port.reset_input_buffer()
        return

#################################################
# handle USB port selection for Arduino port only
#################################################
def get_arduino_ports():
    ports = serial.tools.list_ports.comports(include_links=True)

    num_ports = len(ports)

    arduinos = [None] * num_ports

    for i in range(num_ports):
        if re.search('Arduino',str(ports[i].manufacturer)):
            arduinos[i] = ports[i]
    num_arduinos = 0

    for i in range(len(arduinos)):
        if arduinos[i] != None:
            num_arduinos += 1

    if num_arduinos == 1:
        return arduinos[0]
    elif num_arduinos > 1:
        return
    else:
        return

def main(args=None):
    rclpy.init(args=args)

    serial = Serial()

    rclpy.spin(serial)

if __name__ == '__main__':
    main()
