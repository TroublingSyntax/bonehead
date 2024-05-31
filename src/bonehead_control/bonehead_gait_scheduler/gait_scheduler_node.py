import rclpy
from rclpy.node import Node
from bonehead_msgs.msg import LegCoords
from bonehead_msgs.msg import Control
from bonehead_gait_scheduler.Bezier import Bezier
from bonehead_gait_scheduler.config import BoneheadConfiguration as config

import numpy as np

class GaitScheduler(Node):
    def __init__(self):
        super().__init__('gait_scheduler_node')

        self.bz_ = Bezier()
        self.coords_  = LegCoords()

        self.prev_cmd_ = Control()
        self.this_cmd_ = Control()
        self.config_ = config()

        self.isUpdated_ = False
        self.indices_ = self.config_.walk_start

        self.forward_footpath_ = np.empty((self.config_.n1+self.config_.n2,3))
        self.forward_footpath_ = self.bz_.gen_path(11,self.config_.n1,self.config_.n2,self.config_.forward_ctl_points)
        
        self.reverse_footpath_ = np.empty((self.config_.n1+self.config_.n2,3))
        self.reverse_footpath_ = np.flipud(self.forward_footpath_)

        self.left_strafe_footpath_  = np.empty((self.config_.n1+self.config_.n2,3))
        self.left_strafe_footpath_  = self.bz_.gen_path(11,self.config_.n1,self.config_.n2,self.config_.strafing_ctl_points)

        self.right_strafe_footpath_ = np.empty((self.config_.n1+self.config_.n2,3))
        self.right_strafe_footpath_ = np.flipud(self.left_strafe_footpath_)

        self.publisher_ = self.create_publisher(LegCoords, 'leg_coordinates', 1)
        self.subscriber_ = self.create_subscription(Control, 'control_input', self.sub_callback, 1)
        self.timer_period_ = 1 / self.config_.foot_pub_freq # in seconds
        self.timer_ = self.create_timer(self.timer_period_,self.pub_callback)

    def pub_callback(self):
        for i in range(4):
            if self.indices_[i] >= (self.config_.n1 + self.config_.n2): # overflow handling
                self.indices_[i] = 0
        match self.this_cmd_.state:
            case 0:
                self.coords_.frx = self.config_.stance_coords[0]
                self.coords_.flx = self.config_.stance_coords[0]
                self.coords_.brx = self.config_.stance_coords[0]
                self.coords_.blx = self.config_.stance_coords[0]

                self.coords_.fry = self.config_.stance_coords[1]
                self.coords_.fly = self.config_.stance_coords[1]
                self.coords_.bry = self.config_.stance_coords[1]
                self.coords_.bly = self.config_.stance_coords[1]

                self.coords_.frz = self.config_.stance_coords[2]
                self.coords_.flz = self.config_.stance_coords[2]
                self.coords_.brz = self.config_.stance_coords[2]
                self.coords_.blz = self.config_.stance_coords[2]
            case 1:
                self.update_coords(self.forward_footpath_, 0)
            case 2:
                self.update_coords(self.reverse_footpath_, 0)
            case 3:
                self.update_coords(self.left_strafe_footpath_, 1)
            case 4:
                self.update_coords(self.right_strafe_footpath_, 1)
            case 5:
                self.coords_.frx = self.config_.L3
                self.coords_.flx = self.config_.L3
                self.coords_.brx = self.config_.L3
                self.coords_.blx = self.config_.L3

                self.coords_.fry = self.config_.L1
                self.coords_.fly = self.config_.L1
                self.coords_.bry = self.config_.L1
                self.coords_.bly = self.config_.L1

                self.coords_.frz = self.config_.L2
                self.coords_.flz = self.config_.L2
                self.coords_.brz = self.config_.L2
                self.coords_.blz = self.config_.L2
            case 6: # basically home position, but z coordinates are raised up
                self.coords_.frx = self.config_.L3 + 0.03
                self.coords_.flx = self.config_.L3 + 0.03
                self.coords_.brx = self.config_.L3 + 0.03
                self.coords_.blx = self.config_.L3 + 0.03

                self.coords_.fry = self.config_.L1
                self.coords_.fly = self.config_.L1
                self.coords_.bry = self.config_.L1
                self.coords_.bly = self.config_.L1

                self.coords_.frz = self.config_.L2 - 0.04
                self.coords_.flz = self.config_.L2 - 0.04
                self.coords_.brz = self.config_.L2 - 0.04
                self.coords_.blz = self.config_.L2 - 0.04
                
        for i in range(4):
            self.indices_[i] += 1

        # compensate for roll / pitch commands
        self.coords_.frz = self.coords_.frz + self.this_cmd_.fr_z_comp
        self.coords_.flz = self.coords_.flz + self.this_cmd_.fl_z_comp
        self.coords_.brz = self.coords_.brz + self.this_cmd_.br_z_comp
        self.coords_.blz = self.coords_.blz + self.this_cmd_.bl_z_comp
        
        self.publisher_.publish(self.coords_)
        return

    def update_coords(self, footpath, mode):

        if self.isUpdated_: # if transitioning, restart indices
            self.indices_ = self.config_.start_points[mode]
            self.isUpdated_ = False
        for i in range(4):
            if self.indices_[i] >= (self.config_.n1 + self.config_.n2): # overflow handling
                self.indices_[i] = 0
        if mode == 1: # if strafing, invert have of the y values
            self.coords_.frx = footpath[self.indices_[0],0]
            self.coords_.flx = footpath[self.indices_[1],0]
            self.coords_.brx = footpath[self.indices_[2],0]
            self.coords_.blx = footpath[self.indices_[3],0]

            self.coords_.fry = -footpath[self.indices_[0],1]
            self.coords_.fly = footpath[self.indices_[1],1]
            self.coords_.bry = -footpath[self.indices_[2],1]
            self.coords_.bly = footpath[self.indices_[3],1]

            self.coords_.frz = footpath[self.indices_[0],2]
            self.coords_.flz = footpath[self.indices_[1],2]
            self.coords_.brz = footpath[self.indices_[2],2]
            self.coords_.blz = footpath[self.indices_[3],2]
        else:
            self.coords_.frx = footpath[self.indices_[0],0]
            self.coords_.flx = footpath[self.indices_[1],0]
            self.coords_.brx = footpath[self.indices_[2],0]
            self.coords_.blx = footpath[self.indices_[3],0]

            self.coords_.fry = footpath[self.indices_[0],1]
            self.coords_.fly = footpath[self.indices_[1],1]
            self.coords_.bry = footpath[self.indices_[2],1]
            self.coords_.bly = footpath[self.indices_[3],1]

            self.coords_.frz = footpath[self.indices_[0],2]
            self.coords_.flz = footpath[self.indices_[1],2]
            self.coords_.brz = footpath[self.indices_[2],2]
            self.coords_.blz = footpath[self.indices_[3],2]
        return
            
    def sub_callback(self, command):
        self.prev_cmd_ = self.this_cmd_
        self.this_cmd_ = command
        if self.prev_cmd_ != self.this_cmd_:
            self.isUpdated_ = True
        else:
            self.isUpdated_ = False
        return                

def main(args=None):
    rclpy.init(args=args)
    gait_scheduler = GaitScheduler()
    rclpy.spin(gait_scheduler)
    gait_scheduler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()