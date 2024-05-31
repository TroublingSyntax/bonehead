import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from bonehead_msgs.msg import LegCoords
from bonehead_msgs.msg import ServoCmd
from bonehead_msgs.msg import Control
from bonehead_ik.InverseKinematics import InverseKinematics
from bonehead_gait_scheduler.config import BoneheadConfiguration as config 

import numpy as np

class InvKin_Node(Node):
    def __init__(self):
        super().__init__('ik_node')
        self.IK = InverseKinematics()
        self.coords = LegCoords()
        self.cfg_ = config()
        self.joint_names = ["fr_hip_joint", "fr_ul_joint", "fr_ll_joint", \
                            "fl_hip_joint", "fl_ul_joint", "fl_ll_joint", \
                            "br_hip_joint", "br_ul_joint", "br_ll_joint", \
                            "bl_hip_joint", "bl_ul_joint", "bl_ll_joint"  ]
        self.joint_angles = JointState()
        self.servo_cmds = ServoCmd()
        self.pose_ = Control()
        self.joint_angles.name = ['', '', '', '', '', '', \
                                  '', '', '', '', '', ''  ]
        self.joint_angles.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0  ]
        self.joint_angles.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0  ]
        self.joint_angles.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0  ]
        self.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                       0.0, 0.0, 0.0, 0.0, 0.0, 0.0  ]
        
        # these are for adjusted pwm bounds, bounds can be found in config file
        self.servo_low = 163
        self.servo_high = 420

        self.subscription_ = self.create_subscription(LegCoords, 'leg_coordinates',self.sub_callback, 30)
        self.subscription_ = self.create_subscription(Control, 'control_input', self.pose_callback, 30)
        self.publisher_ = self.create_publisher(JointState, 'servo_angles', 30)
        self.cmd_publisher_ = self.create_publisher(ServoCmd, 'servo_cmd', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.pub_callback)

    def sub_callback(self, leg_coords):

        fr_coord = np.array([leg_coords.frx, leg_coords.fry, leg_coords.frz])
        fl_coord = np.array([leg_coords.flx,leg_coords.fly,leg_coords.flz])
        br_coord = np.array([leg_coords.brx, leg_coords.bry, leg_coords.brz])
        bl_coord = np.array([leg_coords.blx,leg_coords.bly,leg_coords.blz])

        fr_angles = self.IK.get_FR(fr_coord)
        fl_angles = self.IK.get_FL(fl_coord)
        br_angles = self.IK.get_BR(br_coord)
        bl_angles = self.IK.get_BL(bl_coord)

        if any(np.isnan(fr_angles)):
            return
        if any(np.isnan(fl_angles)):
            return
        if any(np.isnan(br_angles)):
            return
        if any(np.isnan(bl_angles)):
            return
        else:
            self.angles = np.concatenate((fr_angles,fl_angles,br_angles,bl_angles),axis=None)
            return

    def pub_callback(self):

        for i in range(0,12):
            self.joint_angles.name[i] = self.joint_names[i]
            self.joint_angles.position[i] = self.angles[i]
            self.joint_angles.velocity[i] = 3.0
            self.joint_angles.effort[i] = 100
            self.joint_angles.header.stamp = self.get_clock().now().to_msg()
            j = 0
            for j in range(len(self.cfg_.right_indices)):
                if i == self.cfg_.right_indices[j]:
                    self.angles[i] = -self.angles[i]
            self.map_servo_offsets(self.cfg_.servo_centers[i]) # compensate for servo calibration
            self.servo_cmds.pwm[i] = self.angle2pwm(self.angles[i], -np.pi/2, np.pi/2, self.servo_low, self.servo_high)

        self.cmd_publisher_.publish(self.servo_cmds)
        self.publisher_.publish(self.joint_angles)
        return
    
    def pose_callback(self, pose):
        self.pose_ = pose
        return
    
    def angle2pwm(self, val, prev_low, prev_high, now_low, now_high):
        prev_range = prev_high - prev_low
        now_range = now_high - now_low
        prev_location = (val - prev_low) / prev_range
        now_location = int((prev_location * now_range) + now_low)
        return now_location
    
    def map_servo_offsets(self, center_point):
        self.servo_low = center_point - 128
        self.servo_high = center_point + 128
        return

def main(args=None):
    rclpy.init(args=args)
    inv_kin = InvKin_Node()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()