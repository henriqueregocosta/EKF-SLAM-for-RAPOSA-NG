#!/usr/bin/env python

import numpy as np
import math

class SLAM(object):
    def __init__(self):
        self.mean_pred = np.zeros(3)
        self.cov_pred = np.zeros((3,3))

    def quaternions(self, qx, qy, qz, qw):
        return math.asin(2*(qw*qy - qz*qx)) 

    def odometry_model(self, theta, odometry):
        x_hat, y_hat, qx, qy, qz, qw = odometry.position_and_quaternions
        
        delta_rot1 = math.atan(y_hat/x_hat) - theta
        delta_trans = math.sqrt(x_hat*x_hat + y_hat*y_hat)
        
        theta_hat = self.quaternions(qx, qy, qz, qw)
        delta_rot2 = theta_hat - theta - delta_rot1
        return delta_rot1, delta_trans, delta_rot2
    
    def update_robot_pos(self, N, theta, odometry, R):
        Fx = np.array([np.identity(3), np.zeros((3,3*N))])

        delta_rot1, delta_trans, delta_rot2 = self.odometry_model(theta, odometry)
        a = delta_trans*math.cos(theta + delta_rot1)
        b = delta_trans*math.sin(theta + delta_rot1)
        c = delta_rot1 + delta_rot2
        self.mean_pred = self.mean_pred + np.dot(Fx.T,np.array([a, b, c]))
        
        g = np.matrix([[0, 0, -b],[0, 0, a],[0, 0, 0]])
        G = np.identity(3*N+3) + np.dot(np.dot(Fx.T,g),Fx)
        self.cov_pred = G.dot(self.cov_pred).dot(G.T) + Fx.T.dot(R).dot(Fx)

    def add_unseen_landmark(self, mean_pred, theta, z):
        update = np.zeros(3)
        update[0] = mean_pred[0] + z[0]*math.sin(theta) + z[1]*math.cos(theta)
        update[1] = mean_pred[1] + z[1]*math.sin(theta) - z[0]*math.cos(theta)
        update[2] = z[2]
        return np.array(list(mean_pred).append(update))

    def predict_landmark_pos(self, j):
        x_lm = self.mean_pred[3+3*(j+1)]
        y_lm = self.mean_pred[3+3*(j+1)+1]
        x_r = self.mean_pred[0]
        y_r = self.mean_pred[1]
        theta_r = self.mean_pred[2]
        
        z_pred = np.zeros(3)
        
        z_pred[0] = (x_lm - x_r)*math.sin(theta_r) - (y_lm - y_r)*math.cos(theta_r)
        z_pred[1] = -(x_lm - x_r)*math.cos(theta_r) + (y_lm - y_r)*math.sin(theta_r)
        z_pred[2] = mean_pred[3+3*(j+1)+2]
        return z_pred

    def update_seen_landmarks(self, j, N, z, Q, R):
        x_lm = self.mean_pred[3+3*(j+1)]
        y_lm = self.mean_pred[3+3*(j+1)+1]
        x_r = self.mean_pred[0]
        y_r = self.mean_pred[1]
        theta_r = self.mean_pred[2]
        z_pred = self.predict_landmark_pos(j)
        
        Fx_j = np.matrix([[np.identity(3), np.zeros((3,3*N))],
            [np.zeros((3,3*(j+1))), np.identity(3), np.zeros((3,3*N-3*(j+1)))]])
        
        h = np.matrix([[-math.sin(theta_r), math.cos(theta_r), (x_lm-x_r)*math.cos(theta_r)+(y_lm-y_r)*math.sin(theta_r), math.sin(theta_r), -math.cos(theta_r), 0],
            [math.cos(theta_r), -math.sin(theta_r), (x_lm-x_r)*math.sin(theta_r)+(y_lm-y_r)*math.cos(theta_r), -math.cos(theta_r), math.sin(theta_r), 0],
            [0, 0, 0, 0, 0, 1]])
        H = np.dot(h, Fx_j)
        
        K = self.cov_pred.dot(H.T).dot(np.linalg.inv(H.dot(self.cov_pred).dot(H.T) + Q))
        
        self.mean_pred += K.dot(z-z_pred)
        self.cov_pred = (np.identity(np.shape(K.dot(H))) - K.dot(H)).dot(self.cov_pred)

    def EKF(self, odometry, observations, Q, R):
        N = (len(self.mean_pred)-3)/3
        theta = self.mean_pred[2]
        
        if odometry.state == 1:
            self.update_robot_pos(N, theta, odometry, R)
            odometry.state = 0
        if observations.state == 1:
            for z in observations.markersisee: # z = [x y s].T
                j = 0
                for i in range(N):
                    if(z[2]==self.mean_pred[5+3*i]):
                        j = i
                if j == 0:
                    self.mean_pred = self.add_unseen_landmark(self.mean_pred, theta, z)
                    j = N
                    N += 1
                self.update_seen_landmarks(j, N, z, Q, R)
            observations.state = 0