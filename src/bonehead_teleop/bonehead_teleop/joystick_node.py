import rclpy
from rclpy.node import Node
from bonehead_msgs.msg import Control
from bonehead_msgs.msg import MotorControl
from bonehead_gait_scheduler import config

import pygame as pg
import numpy as np

import time
import threading
import os

class Joystick(Node):
    def __init__(self):
        os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
        pg.display.init()

        super().__init__('joystick_node')
        self.js_ = None
        self.lock_ = threading.Lock()
        # raw controller inputs
        self.is_walk_ = False
        self.is_strafe_ = False
        self.left_vertical_ = 0
        self.left_horizontal_ = 0
        self.right_horizontal_ = 0
        self.right_vertical_ = 0
        self.curr_roll_cmd_ = 0
        self.curr_pitch_cmd_ = 0

        # startup vars
        self.start_stop_counter = 0

        # shutdown vars
        self.shutting_down = False

        # output commands
        self.pose_ = Control() # default to zero
        self.pose_.state = 5 # start in home position

        self.motors_ = MotorControl() # motor controller message
        self.motors_.long_motion = 0.0
        self.motors_.lat_motion = 0.0

        self.actuation_type = 0 # 0 for quadruped, 1 for motors
        self.cf_ = config.BoneheadConfiguration()

        self.timer_period_ = 0.05
        self.create_timer(self.timer_period_, self.update_buttons)
        self.command_pub_ = self.create_publisher(Control, 'control_input', 1)
        self.motor_command_pub_ = self.create_publisher(MotorControl, 'motor_control_input', 1)

        threading.Thread(target=self.connect_joystick, daemon=True).start()
        

    def connect_joystick(self):
        while True:
            if os.path.exists("/dev/input/js0"):
                with self.lock_:
                    if self.js_ is None:
                        pg.joystick.init()
                        try:
                            self.js_ = pg.joystick.Joystick(0)
                            self.js_.init()
                        except Exception as e:
                            self._logger.error(e)
                            self.js_ = None
            else:
                with self.lock_:
                    if self.js_ is not None:
                        self.js_.quit()
                        self.js_ = None
    def update_buttons(self):
        with self.lock_:
            if self.js_ is None:
                return
            pg.event.pump()
            buttons = [self.js_.get_button(i) for i in range(13)]
            
            if buttons[1] == 1.0:
                self.poll_switch_motors(buttons)

            if buttons[11] == 1.0: # button debounce
                self.start_stop_counter += 1

            if (buttons[6] == 1.0 and buttons[7] == 1.0 and buttons[8] == 1.0 and buttons[9] == 1.0):
                    self.poll_shutdown(buttons)
                

            if self.start_stop_counter >= 50: # if long pressed...
                if self.pose_.state == 0: # and if previously stance...
                    self.pose_.state = 5 # change to home position
                    self.start_stop_counter = 0 # reset counter
                else: # if previously home...
                    self.pose_.state = 0 # change to stance position otherwise
                    self.start_stop_counter = 0 # reset counter

            if self.pose_.state != 5: # if not home position, then read input data
            
                self.left_vertical_ = self.js_.get_axis(1)
                if self.pose_.state == 6: # if motors are active
                    self.motors_.long_motion = self.left_vertical_
                else:
                    if self.left_vertical_ < -0.5:
                        self.pose_.state = 1
                        self.is_walk_ = True
                    elif self.left_vertical_ > 0.5:
                        self.pose_.state = 2
                        self.is_walk_ = True
                    else:
                        self.pose_.state = 0
                        self.is_walk_ = False
                        self.is_strafe_ = False

                self.left_horizontal_ = self.js_.get_axis(0)
                if self.pose_.state == 6: # if motors are active
                    self.motors_.lat_motion = self.left_horizontal_
                else:
                    if self.left_horizontal_ < -0.5:
                        self.pose_.state = 3
                        self.is_strafe_ = True
                    elif self.left_horizontal_ > 0.5:
                        self.pose_.state = 4
                        self.is_strafe_ = True
                    
                    if self.is_walk_ and self.is_strafe_: # if both used, just walk
                        if self.left_vertical_ < -0.5:
                            self.pose_.state = 1
                        elif self.left_vertical_ > 0.5:
                            self.pose_.state = 2
                        else:
                            self.pose_.state = 0
                
                self.right_horizontal_ = self.js_.get_axis(2)
                if self.pose_.state == 6:
                    pass
                else:
                    if np.abs(self.right_horizontal_) < 0.04: self.right_horizontal_ = 0
                    self.curr_roll_cmd_ = self.right_horizontal_ * self.cf_.max_rot_cmd

                self.right_vertical_ = self.js_.get_axis(3)
                if self.pose_.state == 6:
                    pass
                else:
                    if np.abs(self.right_vertical_) < 0.04: self.right_vertical_ = 0
                    self.curr_pitch_cmd_ = self.right_vertical_ * self.cf_.max_rot_cmd

                self.pose_.fr_z_comp = ((-self.cf_.cw/2)*np.sin(self.curr_roll_cmd_)) \
                                    + ((self.cf_.cl/2)*np.sin(self.curr_pitch_cmd_))
                self.pose_.fl_z_comp = ((self.cf_.cw/2)*np.sin(self.curr_roll_cmd_)) \
                                    + ((self.cf_.cl/2)*np.sin(self.curr_pitch_cmd_))
                self.pose_.br_z_comp = ((-self.cf_.cw/2)*np.sin(self.curr_roll_cmd_)) \
                                    + ((-self.cf_.cl/2)*np.sin(self.curr_pitch_cmd_))
                self.pose_.bl_z_comp = ((self.cf_.cw/2)*np.sin(self.curr_roll_cmd_)) \
                                    + ((-self.cf_.cl/2)*np.sin(self.curr_pitch_cmd_))
            self.command_pub_.publish(self.pose_)
            self.motor_command_pub_.publish(self.motors_)

    def poll_shutdown(self, buttons):
        start_time = time.time()
        while ((buttons[6] == 1.0 and buttons[7] == 1.0 and buttons[8] == 1.0 and buttons[9] == 1.0)):
            this_time = time.time()
            elapsed_time = this_time - start_time
            if (elapsed_time >= 5.0):
                os.system("shutdown -h")
                while True:
                    pass
        return
    
    def poll_switch_motors(self, buttons):
            if self.actuation_type == 0: # if in quadruped
                self.pose_.state = 6
                self.actuation_type = 1
            else:
                self.pose_.state = 0 # stance
                self.actuation_type = 0
            time.sleep(1)
            return

def main(args=None):
    rclpy.init(args=args)
    joystick = Joystick()
    rclpy.spin(joystick)
    joystick.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()