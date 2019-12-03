import numpy as np
from numpy.linalg import inv
import math

def check_consistency(e_mean_pred, t_mean_pred, cov_pred):
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

    # Normalized estimation error squared (NEES or D^2)
    D2 = np.dot(np.dot(dx, invcov),dx)
    # print(D2)

    # chi-squared
    # chi_sq = np.sum((dx**2) / x_true)
    # print(chi_sq)


def EKFconsistency(e_mean_pred, cov):

    trajectory = straightline(0,0,0,1,50,'x')
    trajectory.extend(straightline(50, -1, -math.pi/2, -1, 19, 'y'))
    trajectory.extend(straightline(49, -20, math.pi, -1, 99, 'x'))
    trajectory.extend(straightline(-50, -19, math.pi/2, 1, 19, 'y'))
    trajectory.extend(straightline(-50, 0, 0, 1, 49, 'x'))

    check_consistency(e_mean_pred[:3], trajectory, cov)
    print('success')
