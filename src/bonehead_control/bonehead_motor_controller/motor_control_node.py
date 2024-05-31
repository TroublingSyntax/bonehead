import rclpy
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import JointState
from bonehead_msgs.msg import MotorControl
from bonehead_msgs.msg import MotorCmd
from bonehead_msgs.msg import Control
from bonehead_gait_scheduler.config import BoneheadConfiguration as config

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.cf_ = config()

        self.pose_ = Control()
        self.pose_.state = 5 # start in home position

        self.motor_control_ = MotorControl()
        self.motor_control_.long_motion = 0.0
        self.motor_control_.lat_motion = 0.0

        self.motor_cmd_ = MotorCmd()
        self.motor_cmd_.pwm = [0, 0, 0, 0] # default to zero PWM
        self.motor_cmd_.dir = [0, 0, 0, 0] # default to forward

        self.motor_joints_ = JointState() # motors for rviz
        self.motor_joints_.name = ['', '', '', '']
        self.motor_joints_.position = [0.0, 0.0, 0.0, 0.0]
        self.motor_joints_.velocity = [0.0, 0.0, 0.0, 0.0]
        self.motor_joints_.effort = [0.0, 0.0, 0.0, 0.0]
        self.last_time_ = 0.0
        self.this_time_ = 0.0
        
        self.motor_names_ = ["fr_wheel_joint", "fl_wheel_joint", \
                             "br_wheel_joint", "bl_wheel_joint"]

        self.publisher_ = self.create_publisher(MotorCmd, 'motor_cmd', 1)
        self.motor_publisher_ = self.create_publisher(JointState, 'motor_speeds', 30)
        self.subscriber_ = self.create_subscription(Control, 'control_input', self.sub_callback, 1)
        self.motor_sub_ = self.create_subscription(MotorControl, 'motor_control_input', self.motor_sub_callback, 1)
        self.timer_period_ = 1 / self.cf_.foot_pub_freq # in seconds
        self.timer_ = self.create_timer(self.timer_period_,self.pub_callback)

    def pub_callback(self):
        if self.pose_.state != 6: # if not in motor drive mode...
            self.motor_cmd_.pwm = [0, 0, 0, 0]
            self.motor_cmd_.dir = [0, 0, 0, 0]
        else: # if in motor drive mode...
            if self.motor_control_.long_motion < -0.1:
                self.motor_cmd_.pwm = [200, 200, 200, 200] # arbitrary values, for now
                self.motor_cmd_.dir = [0, 0, 0, 0]
                self.this_time_ = self.get_clock().now().to_msg()._nanosec
                for i in range(0,4):
                    self.motor_joints_.name[i] = self.motor_names_[i]
                    self.motor_joints_.position[i] = self.motor_joints_.position[i] + (-7.33 * 
                                                    (self.this_time_ - self.last_time_) / 1000000000)
                    if self.motor_joints_.position[i] > 6.24 or self.motor_joints_.position[i] < -6.24:
                        self.motor_joints_.position[i] = 0
                    self.motor_joints_.velocity[i] = -7.33 # roughly 70 rpm in rad/s
                    self.motor_joints_.effort[i] = 100
                self.motor_joints_.header.stamp = self.get_clock().now().to_msg()
                self.last_time_ = self.this_time_
            elif self.motor_control_.long_motion > 0.1:
                self.motor_cmd_.pwm = [200, 200, 200, 200] # arbitrary values, for now
                self.motor_cmd_.dir = [1, 1, 1, 1]
                self.this_time_ = self.get_clock().now().to_msg()._nanosec
                for i in range(0,4):
                    self.motor_joints_.name[i] = self.motor_names_[i]
                    self.motor_joints_.position[i] = self.motor_joints_.position[i] + (7.33 * 
                                                    (self.this_time_ - self.last_time_) / 1000000000)
                    if self.motor_joints_.position[i] > 6.24 or self.motor_joints_.position[i] < -6.24:
                        self.motor_joints_.position[i] = 0
                    self.motor_joints_.velocity[i] = 7.33 # roughly 70 rpm in rad/s
                    self.motor_joints_.effort[i] = 100
                self.motor_joints_.header.stamp = self.get_clock().now().to_msg()
                self.last_time_ = self.this_time_
            else:
                self.motor_cmd_.pwm = [0, 0, 0, 0] # arbitrary values, for now
                self.motor_cmd_.dir = [2, 2, 2, 2]
                for i in range(0,4):
                    self.motor_joints_.name[i] = self.motor_names_[i]
                    self.motor_joints_.velocity[i] = 0.0 # roughly 70 rpm
                    self.motor_joints_.effort[i] = 100
                self.motor_joints_.header.stamp = self.get_clock().now().to_msg()
        self.motor_publisher_.publish(self.motor_joints_)
        self.publisher_.publish(self.motor_cmd_)

    def sub_callback(self, command):
        self.pose_ = command

    def motor_sub_callback(self, motor_cmd):
        self.motor_control_.long_motion = motor_cmd.long_motion
        self.motor_control_.lat_motion = motor_cmd.lat_motion

def main(args=None):
    rclpy.init(args=args)
    motor_control = MotorController()
    rclpy.spin(motor_control)
    motor_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()