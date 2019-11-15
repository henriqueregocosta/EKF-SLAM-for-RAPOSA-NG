import numpy as np
import math

class SLAM(object):
    def __init__(self):
        self.mean_pred = [[0, 0, 0]]
        self.cov_pred = np.zeros((3,3))


    def sum_to_mean_pred(self, array):
        for i in range(len(self.mean_pred)):
            for j in range(3):
                self.mean_pred[i][j] +=  float(array[3*i+j])


    def update_robot_pos(self, N, theta, odometry, R):
        Fx = np.array([np.identity(3), np.zeros((3,3*N))])

        delta_rot1, delta_trans, delta_rot2 = odometry.odometry_model(theta, odometry)
        a = delta_trans*math.cos(theta + delta_rot1)
        b = delta_trans*math.sin(theta + delta_rot1)
        c = delta_rot1 + delta_rot2
        self.sum_to_mean_pred(np.dot(Fx.T,np.array([a, b, c])))
        
        g = np.matrix([[0, 0, -b],[0, 0, a],[0, 0, 0]])
        G = np.identity(3*N+3) + np.dot(np.dot(Fx.T,g),Fx)
        self.cov_pred = G.dot(self.cov_pred).dot(G.T) + Fx.T.dot(R).dot(Fx)


    def add_unseen_landmark(self, theta, z):
        update = np.zeros(3)
        update[0] = self.mean_pred[0][0] + z[0]*math.sin(theta) + z[1]*math.cos(theta)
        update[1] = self.mean_pred[0][1] + z[1]*math.sin(theta) - z[0]*math.cos(theta)
        update[2] = z[2]
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
        
        z_pred[0] = (x_lm - x_r)*math.sin(theta_r) - (y_lm - y_r)*math.cos(theta_r)
        z_pred[1] = -(x_lm - x_r)*math.cos(theta_r) + (y_lm - y_r)*math.sin(theta_r)
        z_pred[2] = self.mean_pred[j][2]
        return z_pred


    def compute_selection_matrix(self, j, N):    
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


    def update_seen_landmarks(self, j, N, z, Q, R):
        x_lm = self.mean_pred[j][0]
        y_lm = self.mean_pred[j][1]
        x_r = self.mean_pred[0][0]
        y_r = self.mean_pred[0][1]
        theta_r = self.mean_pred[0][2]
        z_pred = self.predict_landmark_pos(j)
        
        Fx_j = self.compute_selection_matrix(j, N)

        h = np.matrix([[-math.sin(theta_r), math.cos(theta_r), (x_lm-x_r)*math.cos(theta_r)+(y_lm-y_r)*math.sin(theta_r), math.sin(theta_r), -math.cos(theta_r), 0],
            [math.cos(theta_r), -math.sin(theta_r), (x_lm-x_r)*math.sin(theta_r)+(y_lm-y_r)*math.cos(theta_r), -math.cos(theta_r), math.sin(theta_r), 0],
            [0, 0, 0, 0, 0, 1]])
        H = np.dot(h, Fx_j)
        
        K = self.cov_pred.dot(H.T).dot(np.linalg.inv(H.dot(self.cov_pred).dot(H.T) + Q))

        self.sum_to_mean_pred(K.dot(np.expand_dims(z-z_pred, axis=1)))
        self.cov_pred = (np.identity(len(K.dot(H))) - K.dot(H)).dot(self.cov_pred)


    def EKF(self, odometry, observations, Q, R):
        N = len(self.mean_pred) - 1 # nr of landamrks already seen
        theta = self.mean_pred[0][2]
        
        if odometry.state == 1:
            print('odometry')
            self.update_robot_pos(N, theta, odometry, R)
            odometry.state = 0
        if observations.state == 1:
            for z in observations.markersisee: # z = [x y s].T
                j = 0
                if N>0: # check if the observation corresponds to a already seen landmark
                    for i in range(N):
                        if(z[2]==self.mean_pred[i+1][2]):
                            j = i+1
                if j == 0:
                    self.add_unseen_landmark(theta, z)
                    N += 1
                    j = N
                self.update_seen_landmarks(j, N, z, Q, R)
            observations.state = 0
            print(self.mean_pred)