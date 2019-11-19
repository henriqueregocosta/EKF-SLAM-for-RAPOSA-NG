import math
import multiprocessing
import numpy as np
import _thread
import rospy

class SLAM(object):
    def __init__(self, queue_name):
        self.mean_pred = [[0, 0, 0]]
        self.cov_pred = np.zeros((3,3))

        self.q = queue_name


    def quaternions(self, qx, qy, qz, qw):
        return math.asin(2*(qx*qz - qw*qy)) 


    def odometry_model(self, theta, odometry):
        x_hat, y_hat, qx, qy, qz, qw = odometry
        
        delta_rot1 = math.atan2(y_hat,x_hat) - theta
        delta_trans = math.sqrt(x_hat*x_hat + y_hat*y_hat)
        
        theta_hat = self.quaternions(qx, qy, qz, qw)
        delta_rot2 = theta_hat - theta - delta_rot1
        return delta_rot1, delta_trans, delta_rot2


    def sum_to_mean_pred(self, array):
        for i in range(len(self.mean_pred)):
            for j in range(3):
                self.mean_pred[i][j] += float(array[3*i+j]


    def update_robot_pos(self, event):
        N = len(self.mean_pred) - 1
        theta = self.mean_pred[0][2]

        Fx = np.bmat([np.identity(3), np.zeros((3,3*N))])

        delta_rot1, delta_trans, delta_rot2 = self.odometry_model(theta, event[1])
        a = delta_trans*math.cos(theta + delta_rot1)
        b = delta_trans*math.sin(theta + delta_rot1)
        c = delta_rot1 + delta_rot2

        print('pose in odometry')
        print([self.mean_pred[0][0], self.mean_pred[0][1], self.mean_pred[0][2]])

        self.sum_to_mean_pred(np.dot(Fx.T,np.array([[a], [b], [c]])))
        
        g = np.matrix([[0, 0, -b],[0, 0, a],[0, 0, 0]])
        G = np.identity(3*N+3) + np.dot(np.dot(Fx.T,g),Fx)
        self.cov_pred = G.dot(self.cov_pred).dot(G.T) + Fx.T.dot(event[2]).dot(Fx)


    def search_for_landmark(self, z):
        N = len(self.mean_pred) - 1
        j = 0
        if N>0: # check if the observation corresponds to a already seen landmark
            for i in range(N):
                if(z[2]==self.mean_pred[i+1][2]):
                    j = i+1
        return j


    def add_unseen_landmark(self, z):
        theta = self.mean_pred[0][2]

        update = np.zeros(3)
        update[0] = self.mean_pred[0][0] + z[0]*math.cos(theta) - z[1]*math.sin(theta)
        update[1] = self.mean_pred[0][1] + z[0]*math.sin(theta) + z[1]*math.cos(theta)
        update[2] = z[2]
        
        self.mean_pred.append(list(update))
        self.cov_pred = np.bmat([[self.cov_pred, np.zeros((len(self.cov_pred),3))],
                                    [np.zeros((3,len(self.cov_pred))), np.identity(3)]]).A

        # debug
        print('update')
        print([update[0],update[1],update[2]])


    def predict_landmark_pos(self, j):
        x_lm = self.mean_pred[j][0]
        y_lm = self.mean_pred[j][1]
        x_r = self.mean_pred[0][0]
        y_r = self.mean_pred[0][1]
        theta_r = self.mean_pred[0][2]
        
        z_pred = np.zeros(3)

        print('x_lm, y_lm')
        print([x_lm, y_lm])
        print('x_r, y_r')
        print([x_r, y_r])
        
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
        
        K = self.cov_pred.dot(H.T).dot(np.linalg.inv(H.dot(self.cov_pred).dot(H.T) + Q))

        self.sum_to_mean_pred(K.dot(np.expand_dims(z-z_pred, axis=1)))
        self.cov_pred = (np.identity(len(K.dot(H))) - K.dot(H)).dot(self.cov_pred)

    def EKF(self):

        event = self.q.get(timeout=10)    # event = ['obs', markers_I_see, Q]
                                # ou event = ['odo', position_and_quaternions, R]
        if event[0] == 'odo': # precisa de R e position_and_quaternions
            print('odometry')
            self.update_robot_pos(event) 
        elif event[0] == 'obs': # precisa de markers_I_see, Q
            print('observations')
            for z in event[1]: # z = [x y s].T
                j = self.search_for_landmark(z)
                if j == 0:
                    self.add_unseen_landmark(z)
                    j = len(self.mean_pred) - 1
                self.update_seen_landmarks(j, z, event[2])
