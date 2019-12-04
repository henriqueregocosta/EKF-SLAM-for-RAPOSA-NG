import math
import numpy as np
import rospy
from numpy.linalg import multi_dot



class SLAM(object):
    def __init__(self, queue_name):
        self.mean_pred = [[0, 0, 0]]
        self.cov_pred = np.zeros((3,3))
        self.landmarks_index = {}

    def sum_to_mean_pred(self, array):
        for i in range(len(self.mean_pred)):
            for j in range(3):
                self.mean_pred[i][j] += float(array[3*i+j])

    def update_robot_pos(self, event):
        N = len(self.mean_pred) - 1
        theta = self.mean_pred[0][2]

        Fx = np.bmat([np.identity(3), np.zeros((3,3*N))])

        delta_rot1, delta_trans, delta_rot2 = event[1]
        a = delta_trans*math.cos(theta + delta_rot1)
        b = delta_trans*math.sin(theta + delta_rot1)
        c = delta_rot1 + delta_rot2


        self.sum_to_mean_pred(np.dot(Fx.T,np.array([[a], [b], [c]])))

        if self.mean_pred[0][2] < -math.pi:
            self.mean_pred[0][2] += 2*math.pi
        if self.mean_pred[0][2] > math.pi:
            self.mean_pred[0][2] += -2*math.pi
        
        g = np.matrix([[0, 0, -b],[0, 0, a],[0, 0, 0]])
        G = np.identity(3*N+3) + multi_dot([Fx.T, g, Fx])
        self.cov_pred = multi_dot([G, self.cov_pred, G.T]) + multi_dot([Fx.T, event[2], Fx])

    def search_for_landmark(self, z):
        if z[2] in self.landmarks_index:
            return self.landmarks_index[z[2]]
        else:
            return 0

    def add_unseen_landmark(self, z):
        theta = self.mean_pred[0][2]

        update = np.zeros(3)
        update[0] = self.mean_pred[0][0] + z[0]*math.cos(theta) - z[1]*math.sin(theta)
        update[1] = self.mean_pred[0][1] + z[0]*math.sin(theta) + z[1]*math.cos(theta)
        update[2] = z[2]

        self.landmarks_index[z[2]] = len(self.mean_pred)
        self.mean_pred.append(list(update))
        self.cov_pred = np.bmat([[self.cov_pred, np.zeros((len(self.cov_pred),3))],
                                    [np.zeros((3,len(self.cov_pred))), np.identity(3)]]).A

    def predict_landmark_pos(self, j):
        x_lm = self.mean_pred[j][0]
        y_lm = self.mean_pred[j][1]
        x_r = self.mean_pred[0][0]
        y_r = self.mean_pred[0][1]
        theta_r = self.mean_pred[0][2]

        z_pred = np.zeros(3)

        z_pred[0] = (x_lm - x_r)*math.cos(theta_r) + (y_lm - y_r)*math.sin(theta_r)
        z_pred[1] = -(x_lm - x_r)*math.sin(theta_r) + (y_lm - y_r)*math.cos(theta_r)
        z_pred[2] = self.mean_pred[j][2]
        return z_pred

    def selection_matrix(self, j, N):    
        if j== N:
            if N == 1:
                Fx_j = np.identity(6)
            else:
                Fx_j = np.bmat([[np.identity(3), np.zeros((3,3*j))],
                                [np.zeros((3,3*j)), np.identity(3)]]).A
        elif j==1:
            Fx_j = np.bmat([np.identity(6), np.zeros((6,3*N-3*j))]).A
        else:
            Fx_j = np.bmat([[np.identity(3), np.zeros((3,3*N))],
                             [np.zeros((3,3*j)), np.identity(3), np.zeros((3,3*N-3*j))]]).A
        return Fx_j

    def jacobian(self, j):
        x_lm = self.mean_pred[j][0]
        y_lm = self.mean_pred[j][1]
        x_r = self.mean_pred[0][0]
        y_r = self.mean_pred[0][1]
        theta_r = self.mean_pred[0][2]

        h = np.matrix([[-math.cos(theta_r), -math.sin(theta_r), -(x_lm-x_r)*math.sin(theta_r)+(y_lm-y_r)*math.cos(theta_r), math.cos(theta_r), math.sin(theta_r), 0],
            [math.sin(theta_r), -math.cos(theta_r), -(x_lm-x_r)*math.cos(theta_r)-(y_lm-y_r)*math.sin(theta_r), -math.sin(theta_r), math.cos(theta_r), 0],
            [0, 0, 0, 0, 0, 1]])
        return h

    def update_seen_landmarks(self, j, z, Q):
        N = len(self.mean_pred) - 1
        
        z_pred = self.predict_landmark_pos(j)
        
        Fx_j = self.selection_matrix(j, N)

        h = self.jacobian(j)
        H = np.dot(h, Fx_j)
        
        K = multi_dot([self.cov_pred, H.T, np.linalg.inv(multi_dot([H, self.cov_pred, H.T]) + Q)])

        self.sum_to_mean_pred(np.dot(K, np.expand_dims(z-z_pred, axis=1)))
        self.cov_pred = np.dot(np.identity(len(np.dot(K, H))) - np.dot(K, H), self.cov_pred)

    def EKF(self, event):
        if event[0] == 'odo': # precisa de R e position_and_quaternions
            self.update_robot_pos(event)
            return False

        elif event[0] == 'obs': # precisa de s_I_see, Q
            for z in event[1]: # z = [x y s].T
                j = self.search_for_landmark(z)
                if j == 0:
                    self.add_unseen_landmark(z)
                    j = len(self.mean_pred) - 1
                self.update_seen_landmarks(j, z, event[2])
            return False

        elif event[0] == 'end':
            return True