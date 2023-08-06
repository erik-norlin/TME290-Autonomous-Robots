import numpy as np
from behavior import *
import csv


class Predict():
	def __init__(self, dt, R):
		self.behavior = Behavior(dt, R)
		self.dt = dt
		self.Q = np.array([[1, 0, 0, 0],
						   [0, 0.23, 0, 0],
						   [0, 0, 0.24, -0.2],
						   [0, 0, -0.2, 0.29]])


	def predict(self, estimate, t):
		v, phi, x, y, P = estimate.get_est()
		v, phiDot = self.behavior.get_v_phiDot(t)

		v_ = v
		phi_ = phi + phiDot*self.dt
		x_ = x + v*np.cos(phi)*self.dt
		y_ = y + v*np.sin(phi)*self.dt
		
		F_ = self.get_F(v, phi)
		P_ = F_ @ P @ F_.T + self.Q
		
		pred = (v_, phi_, x_, y_, P_)
		return pred


	def get_F(self, v, phi):

		a31 = np.cos(phi)*self.dt
		a32 = -v*np.sin(phi)*self.dt
		a41 = np.sin(phi)*self.dt
		a42 = v*np.cos(phi)*self.dt

		F = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [a31.item(), a32.item(), 1, 0],
                      [a41.item(), a42.item(), 0, 1]])		  
		return F