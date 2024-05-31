import numpy as np

class BoneheadConfiguration():
    def __init__(self):

        """ ROBOT GEOMETRY """
        self.L1 = 0.055 # m
        self.L2 = 0.139 # m
        self.L3 = 0.148 # m

        self.cl = 0.3 # m, chassis length
        self.cw = 0.12 # m, chassis width

        self.right_indices = np.array([0, 1, 2, 6, 7, 8]) # for flipping angles on right side

        """ SERVO HARDWARE OFFSETS
        order is fr1, fr2, fr3, 
                 fl1, fl2, fl3,
                 br1, br2, br3,
                 bl1, bl2, bl3
        """
        self.servo_centers = np.array([330, 298, 290,
                                       288, 292, 282,
                                       268, 290, 280,
                                       340, 296, 328])
        
        """ GAIT SCHEDULER PARAMETERS """
        self.stance_coords = np.array([0, self.L1, 0.2])
        self.swing_time = 0.25
        self.stance_time = 0.75
        self.foot_pub_freq = 200 # Hz

        self.max_rot_cmd = (30*np.pi) / 180 # limit of how far controller can roll / pitch robot (radians)

        """self.forward_ctl_points = np.array([[-0.065, self.L1, 0.22], \
                                            [-0.08,  self.L1, 0.22], \
                                            [-0.095, self.L1, 0.155],\
                                            [-0.095, self.L1, 0.155],\
                                            [-0.095, self.L1, 0.155],\
                                            [0,      self.L1, 0.155],\
                                            [0,      self.L1, 0.155],\
                                            [0,      self.L1, 0.15],\
                                            [0.095,  self.L1, 0.15], \
                                            [0.095,  self.L1, 0.15], \
                                            [0.08,   self.L1, 0.24], \
                                            [0.075,  self.L1, 0.24]])"""
        
        self.forward_ctl_points = np.array([[-0.065, self.L1, 0.22], \
                                            [-0.08,  self.L1, 0.22], \
                                            [-0.095, self.L1, 0.155],\
                                            [-0.095, self.L1, 0.155],\
                                            [-0.095, self.L1, 0.155],\
                                            [0,      self.L1, 0.155],\
                                            [0,      self.L1, 0.155],\
                                            [0,      self.L1, 0.15],\
                                            [0.095,  self.L1, 0.15], \
                                            [0.095,  self.L1, 0.15], \
                                            [0.08,   self.L1, 0.22], \
                                            [0.075,  self.L1, 0.24]])
        
        self.strafing_ctl_points = np.array([[0, -0.02, 0.15], \
                                             [0, -0.03,  0.17], \
                                             [0, -0.04, 0.1 ], \
                                             [0, -0.04, 0.1 ], \
                                             [0, -0.04, 0.1 ], \
                                             [0, 0,      0.1 ], \
                                             [0, 0,      0.1 ], \
                                             [0, 0,      0.01], \
                                             [0, 0.075,  0.01], \
                                             [0, 0.075,  0.01], \
                                             [0, 0.06,   0.17], \
                                             [0, 0.055,  0.15]])
        

        self.n1 = int(self.swing_time * self.foot_pub_freq) # number of points in swing phase
        self.n2 = int(self.stance_time * self.foot_pub_freq) # number of points in stance phase

        self.walk_fl_start = 0
        self.walk_br_start = int(0.25 * (self.n1 + self.n2))
        self.walk_fr_start = int(0.5  * (self.n1 + self.n2))
        self.walk_bl_start = int(0.75 * (self.n1 + self.n2))
        self.walk_start = np.array([self.walk_fr_start, self.walk_fl_start, \
                                 self.walk_br_start, self.walk_bl_start ])
        
        self.strafe_fl_start = 0
        self.strafe_br_start = 0
        self.strafe_fr_start = int(0.5  * (self.n1 + self.n2))
        self.strafe_bl_start = int(0.5 * (self.n1 + self.n2))
        self.strafe_start = np.array([self.strafe_fr_start, self.strafe_fl_start, \
                                      self.strafe_br_start, self.strafe_bl_start ])

        self.start_points = np.array([self.walk_start, self.strafe_start])