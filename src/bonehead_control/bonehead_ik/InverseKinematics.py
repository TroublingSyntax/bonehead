import numpy as np
from bonehead_gait_scheduler.config import BoneheadConfiguration as config


class LegStates():
    prev_coord = None
    now_coord  = None
    prev_angles = None
    now_angles = None
    in_singularity = None

class Legs():
    FR = LegStates()
    FL = LegStates()
    BR = LegStates()
    BL = LegStates()

class InverseKinematics():
    def __init__(self):
        self.cf_ = config()
        self.legState = Legs()

    def get_joints(self, coord):

        joint_ang = np.zeros([3])

        r = np.sqrt((coord[1])**2 + (coord[2]**2))
        th1 = np.arctan2(coord[2],coord[1])
        th2 = np.arccos(self.cf_.L1/r)
        joint_ang[0] = th2 - th1

        rprime = np.sqrt(coord[0]**2 + r**2)
        th = np.arctan2(coord[0],r)
        phi = np.arccos((self.cf_.L2**2+rprime**2-self.cf_.L3**2)/(2*self.cf_.L2*rprime))
        joint_ang[1] = phi - th
        psi = np.arccos((self.cf_.L2**2+self.cf_.L3**2-rprime**2)/(2*self.cf_.L2*self.cf_.L3))
        joint_ang[2] = psi - np.pi/2

        return joint_ang
    
    def get_FR(self, coord):
        self.legState.FR.now_angles = self.get_joints(coord)
        self.legState.FR.prev_angles = self.legState.FR.now_angles
        return self.legState.FR.now_angles
    
    def get_FL(self, coord):
        self.legState.FL.now_angles = self.get_joints(coord)
        self.legState.FL.prev_angles = self.legState.FL.now_angles
        return self.legState.FL.now_angles
    
    def get_BR(self, coord):
        self.legState.BR.now_angles = self.get_joints(coord)
        self.legState.BR.now_angles[0] = -1 * self.legState.BR.now_angles[0] # invert for physical robot
        self.legState.BR.prev_angles = self.legState.BR.now_angles
        return self.legState.BR.now_angles
    
    def get_BL(self, coord):
        self.legState.BL.now_angles = self.get_joints(coord)
        self.legState.BL.now_angles[0] = -1 * self.legState.BL.now_angles[0] # invert for physical robot
        self.legState.BL.prev_angles = self.legState.BL.now_angles
        return self.legState.BL.now_angles