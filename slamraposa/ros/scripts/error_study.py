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

    print('Maximum pose error')
    print(np.amax(dx))

    # Normalized estimation error squared (NEES or D^2)
    # D2 = np.dot(np.dot(dx, invcov),dx)
    # print(D2)

    # chi-squared
    # chi_sq = np.sum((dx**2) / x_true)
    # print(chi_sq)


def EKFconsistency(e_mean_pred, cov):

    t_mean_pred = np.asarray([-1.0000000000000009, -1.4101067586952727*10**-15, -8.086623206605827*10**-18, 23.0, -10.0, 4.0, 40.0, -10.0, 5.0, 40.0, 10.0, 102.0, 35.0, -30.0, 203.0, 10.0, -30.0, 202.0, 5.9999999999999991, -10.000000000000002, 3.0, -11.0, -10.000000000000002, 2.0, -15.0, -30.0, 201.0, -28.0, -10.000000000000002, 1.0, -40.0, -30.0, 200.0, -45.0, -10.000000000000002, 0.0, -40.0, 9.9999999999999982, 100.0, -3.6979673245513473*10**-16, 9.9999999999999982, 101.0])
    check_consistency(e_mean_pred, t_mean_pred, cov)
    print('success')
