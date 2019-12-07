import matplotlib.pyplot as plt
import numpy as np
from math import pi, cos, sin
import math


def landmarks_and_path(x,y, mean_pred, cov, plot_type, world):

    plt.figure()
    if world == 'synthetic':
        plt.xlim((-60,60))
        plt.ylim((-60,60))
    elif world == 'real':
        plt.xlim((-10,10))
        plt.ylim((-10,10))

    for i in range(1, len(mean_pred)):
        plt.plot(mean_pred[i][0],mean_pred[i][1], 'r+')
        
    plt.plot(x, y, '-b')
    plt.title('EKF-SLAM estimated path and landmarks position')
    plt.xlabel('x')
    plt.ylabel('y')

    if plot_type == 'include uncertainty':

        last_cov = np.asarray(cov)
        mpred_vec = np.hstack(mean_pred)

        for i in range(3, len(mpred_vec), 3):
            u=mpred_vec[i]                      #x-position of the center
            v=mpred_vec[i+1]                    #y-position of the center
            a=math.sqrt(last_cov[i][i])         #radius on the x-axis
            b=math.sqrt(last_cov[i+1][i+1])     #radius on the y-axis
            t_rot=0                             #rotation angle

            t = np.linspace(0, 2*pi, 100)
            Ell = np.array([a*np.cos(t) , b*np.sin(t)])  
                 #u,v removed to keep the same center location
            R_rot = np.array([[cos(t_rot) , -sin(t_rot)],[sin(t_rot) , cos(t_rot)]])  
                 #2-D rotation matrix

            Ell_rot = np.zeros((2,Ell.shape[1]))
            for i in range(Ell.shape[1]):
                Ell_rot[:,i] = np.dot(R_rot,Ell[:,i])

            plt.plot( u+Ell_rot[0,:] , v+Ell_rot[1,:],'aqua' )    #rotated ellipse
    
    plt.grid(color='lightgray',linestyle='--')
    plt.show(block=False)

def cov_time(pose_cov, nr_landmarks_seen, dist):
    
    pose_cov = np.asarray(pose_cov)
    nr_landmarks_seen = np.asarray(nr_landmarks_seen)
    dist = np.asarray(dist)
    k = np.arange(0,len(pose_cov))

    plt.figure()
    ldmkp = plt.plot(k, nr_landmarks_seen, 'p', label='nr_landmarks_seen')
    distp = plt.plot(k, dist, 'g', label='dist')
    covxp = plt.plot(k, pose_cov[:,0,0], 'b', label='cov_x')
    covyp = plt.plot(k, pose_cov[:,1,1], 'r', label='cov_y')
    covheadingp = plt.plot(k, pose_cov[:,2,2], 'aqua', label='cov_heading')
    plt.legend(loc="upper right")
    plt.title('Pose covariance over time')
    plt.xlabel('k (time step)')
    plt.ylabel('cov(k)')
    plt.show(block=False)

def variable_individual_analysis(x_odo, y_odo, theta_odo, x, y, theta):

    # analysis of single parameters
    k = np.arange(0,len(x_odo))
    k1 = np.arange(0,len(x))

    plt.figure()
    plt.plot(k, theta_odo, '-k', label='heading odometry')
    plt.plot(k1, theta, '-.r', label='heading EKF-SLAM')
    plt.legend(loc="upper right")
    plt.title('Variable evolution over time')
    plt.show(block=False)

    plt.figure()
    plt.plot(k, x_odo, '-k', label='x-coordinate odometry')
    plt.plot(k1, x, '-.r', label='x-coordinate EKF-SLAM')
    plt.legend(loc="upper right")
    plt.title('Variable evolution over time')
    plt.show(block=False)

    plt.figure()    
    plt.plot(k, y_odo, '-k', label='y-coordinate odometry')
    plt.plot(k1, y, '-.r', label='y-coordinate EKF-SLAM')

    plt.legend(loc="upper right")
    plt.title('Variable evolution over time')
    plt.show(block=False)

def odometry(x_odo, y_odo, theta_odo, world):
    
    plt.figure()
    if world == 'synthetic':
        plt.xlim((-60,60))
        plt.ylim((-60,60))
    elif world == 'real':
        plt.xlim((-10,10))
        plt.ylim((-10,10))

    plt.plot(x_odo,y_odo)
    plt.title('Odometry estimated path')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color='lightgray',linestyle='--')
    plt.show(block=False)
