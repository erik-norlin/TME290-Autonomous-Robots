import csv
import numpy as np

class Update(): 
    def __init__(self, dt):
        self.t_s = [] # t[0] = 0.105
        self.x_s = []
        self.y_s = []
        self.readCSV() # 390 values

        self.dt = dt
        self.H = np.zeros((4,4))
        self.H[2,2] = 1
        self.H[3,3] = 1
        self.R = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 0.49, 0.08],
                           [0, 0, 0.08, 0.67]])


    def update(self, pred, estimate, t, test): # x_ = prediction, x = estimate, x_z = sensor values
        dt = self.dt
        H = self.H

        # Get preds
        v_ = pred[0]
        phi_ = pred[1]
        x_ = pred[2]
        y_ = pred[3]
        P_ = pred[4]

        # Get prev estimates
        v = estimate.v[-1]
        phi = estimate.phi[-1]
        x = estimate.x[-1]
        y = estimate.y[-1]
        P = estimate.P[-1]

        # Get sensor values
        t_steps = int(t/dt)
        if t_steps < len(self.x_s):
            x_z = self.x_s[t_steps]
            y_z = self.y_s[t_steps]
        else:
            x_z = self.x_s[-1]
            y_z = self.y_s[-1]


        # Kalman gain:      K = PH^T(HPH^T + R)^-1
        K = P @ H.T @ np.linalg.pinv(H @ P_ @ H.T + self.R)


        # Update estimates:     X = X + K(Z - HX)
        X_pred = np.array([[v_, phi_, x_, y_]]).T
        Z = np.array([[0, 0, x_z, y_z]]).T
        X_est = np.array([[v, phi, x, y]]).T

        X_est = X_est + K @ (Z - H @ X_pred)


        v = X_est[0,0]
        phi = X_est[1,0]
        x = X_est[2,0]
        y = X_est[3,0]
        P = (np.eye(4) - K @ H) @ P_


        # Kinematic model test
        if test:
            v = v_
            phi = phi_
            x = x_
            y = y_
            P = P_

        est = (v, phi, x, y, P, t)
        return est


    def readCSV(self):
        path = r'data/gnss-1.csv'
        line = 0
        with open(path) as file:
            reader = csv.reader(file)
            for row in reader:
                if line > 0:
                    self.t_s.append(float(row[0]))
                    self.x_s.append(float(row[1]))
                    self.y_s.append(float(row[2]))
                line += 1