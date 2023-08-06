import numpy as np

class Estimate():
    def __init__(self):
        self.v = [0]
        self.phi = [0]
        self.x = [0]
        self.y = [0]
        self.P = [np.zeros((4,4))]
        self.t = [0]


    def set_est(self, est):
        self.v.append(est[0])
        self.phi.append(est[1])
        self.x.append(est[2])
        self.y.append(est[3])
        self.P.append(est[4])
        self.t.append(est[5])

    
    def get_est(self):
        return self.v[-1], self.phi[-1], self.x[-1], self.y[-1], self.P[-1]

    
    def save(self):
        path = r'plot/kinematic_states/'
        np.save(path+'v.npy', np.array(self.v))
        np.save(path+'phi.npy', np.array(self.phi))
        np.save(path+'x.npy', np.array(self.x))
        np.save(path+'y.npy', np.array(self.y))
        np.save(path+'P.npy', np.array(self.P))
        np.save(path+'t.npy', np.array(self.t))
        print('Trajectory saved!')