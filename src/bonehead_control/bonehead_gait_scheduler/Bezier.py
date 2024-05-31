import numpy as np

class Bezier():
    def __init__(self):
        # garbage initialization values
        self.curve_ = np.array([1, 2, 3]) # holds the (x,y,z) pairs of Bezier curve
        self.flat_ = np.array([3, 6, 9]) # holds the (x,y,z) pairs of the flat curve
        self.path_ = np.array([2, 4, 6]) # holds the whole (x,y,z) curve
        self.t_ = np.array([4, 5, 6]) # holds the t values, 0 <= t <= 1
        self.pairs_ = np.array([[-0.035,0.04,0.12], [-0.045,0.04,0.12], [-0.053,0.04,0.085], [-0.053,0.04,0.085], [-0.053,0.04,0.085], [0.0,0.04,0.085],   \
                                [0.0,0.04,0.085], [0.0,0.04,0.08], [0.053,0.04,0.08], [0.053,0.04,0.08], [0.045,0.04,0.12], [0.035,0.04,0.12]])
        self.n_ = 2 # nth order curve
        self.pairs_size_ = 3 # number of pairs for the curve
        self.invalid_combo = False

    def set_order(self, order):
        # simply sets the order of the curve
        self.n_ = order
        return
    
    def set_t(self, num_steps):
        # generates the array t
        self.t_ = np.linspace(0, 1, num=num_steps)
        self.curve_ = np.empty((num_steps, 3))
        return
    
    def set_pairs(self, pairs):
        # sets number of pairs, and bring those pairs into object
        self.pairs_size_ = len(pairs)
        self.pairs_ = np.empty((self.pairs_size_,3))
        self.pairs_ = pairs
        return
    
    def gen_flat(self, num_steps):
        # generates the flat portion of the curve
        self.flat_ = np.linspace(self.curve_[-1],self.curve_[0],num=num_steps)
    
    def bezier(self, pairs, t):
        # generates the (x,y) coordinate for t
        size = len(pairs)
        if size == 2:
            return (1-t) * pairs[0] + t * pairs[1]
        return (1-t) * self.bezier(pairs[:size-1], t) + t * self.bezier(pairs[1:], t)
    
    def update_curve(self):
        # creates the curve points
        if self.invalid_combo:
            return np.array([[0,0],[0,0],[0,0]])
        for i in range(0, len(self.t_)):
            self.curve_[i] = self.bezier(self.pairs_, self.t_[i])
        return self.curve_
    
    def gen_curve(self, n, num_steps, pairs):
        # performs the entire process of curve generation
        self.set_order(n)
        self.set_t(num_steps)
        self.set_pairs(pairs)
        if len(self.pairs_) != (self.n_ + 1):
            print('Invalid pair number and order!\n')
            self.invalid_combo = True
        else:
            self.invalid_combo = False
        self.update_curve()

    def gen_path(self, n, num_steps_curve, num_steps_flat, pairs):
        # performs the entire process of path generation
        self.gen_curve(n, num_steps_curve, pairs)
        self.gen_flat(num_steps_flat)
        curve_length = len(self.curve_)
        flat_length = len(self.flat_)
        self.path_ = np.empty((curve_length + flat_length, 3))
        self.path_ = np.concatenate((self.curve_,self.flat_),axis=0)
        return self.path_