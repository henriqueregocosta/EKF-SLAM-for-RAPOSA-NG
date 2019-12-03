import numpy as np
from numpy.linalg import inv
from numpy.linalg import multi_dot
import matplotlib.pyplot as plt
import math

def check_consistency(x, y, theta, t_mean_pred, cov_pred):
    # =======================================
    # this function should receive a current belief and
    # information regarding groud truth and 
    # check the EKF consistency
    # =======================================

    x_estimated = np.hstack(e_mean_pred)
    x_true = np.hstack(t_mean_pred)
    cov = np.asarray(cov_pred)

    dx = x_true-x_estimated
    invcov = inv(cov)

    print('Maximum pose error')
    print(np.amax(dx))

    # Normalized estimation error squared (NEES or D^2)
    D2 = np.dot(np.dot(dx, invcov),dx)
    # print(D2)

    # chi-squared
    # chi_sq = np.sum((dx**2) / x_true)
    # print(chi_sq)


def straightline(x, y, theta, step, n, direction):
    # -------------
    # x, y, theta first points
    # -------------

    # if direction is 1 direction is y, 0 otherwise
    if direction=='x':
        inc_x=1
        inc_y=0
    elif direction == 'y':
        inc_x=0
        inc_y=1
    elif direction == 'angle':
        inc_x=math.cos(theta)
        inc_y=math.sin(theta)


    trajectory=[[0, 0, 0]]
    trajectory[0][0] = x
    trajectory[0][1] = y
    trajectory[0][2] = theta

    for i in range(n):
        point=[0, 0, 0]
        point[0]=trajectory[i][0]+inc_x*step
        point[1]=trajectory[i][1]+inc_y*step
        point[2]=trajectory[i][2]
        trajectory.append(point)

    return trajectory


def EKFconsistency(x, y, theta, cov, trajectory):
    x = np.expand_dims(np.asarray(x), axis=1)
    y = np.expand_dims(np.asarray(y), axis=1)
    theta = np.expand_dims(np.asarray(theta), axis=1)
    trajectory = np.asarray(x)

    x_estimated = np.hstack((x, y, theta))
    x_true = np.asarray(trajectory)
    cov = np.asarray(cov)

    dx = x_true-x_estimated

    D2 = []
    for dx_t,cov_t in zip(dx[1:],cov[1:]):
        invcov = inv(cov_t)
        D2.append(multi_dot([dx_t.T,invcov,dx_t]))

    # print('Maximum pose error')
    # print(np.amax(dx))

    # Normalized estimation error squared (NEES or D^2)
    k = np.arange(0,len(D2))
    plt.plot(k, D2)
    plt.plot()
    plt.show()