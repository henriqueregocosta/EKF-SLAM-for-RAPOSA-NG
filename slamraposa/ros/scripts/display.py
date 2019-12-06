import matplotlib.pyplot as plt
import numpy as np
from math import pi, cos, sin
import math


def landmarks_and_path(x,y, mean_pred, cov, plot_type):
    plt.xlim((-60,60))
    plt.ylim((-60,60))

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
    plt.show()

    return

def cov_time(pose_cov):
    
    pose_cov = np.asarray(pose_cov)
    k = np.arange(0,len(pose_cov))
    print(np.shape(pose_cov))

    plt.figure()
    covxp = plt.plot(k, pose_cov[:,0,0], 'b', label='cov_x')
    covyp = plt.plot(k, pose_cov[:,1,1], 'r', label='cov_y')
    covheadingp = plt.plot(k, pose_cov[:,2,2], 'aqua', label='cov_heading')
    plt.legend(loc="upper right")
    plt.title('Pose covariance over time')
    plt.xlabel('k (time step)')
    plt.ylabel('cov(k)')
    plt.show()