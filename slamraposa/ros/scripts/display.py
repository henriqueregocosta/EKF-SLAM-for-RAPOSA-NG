import matplotlib.pyplot as plt
import numpy as np
from math import pi, cos, sin
import math


def landmarks_and_path(x,y, mean_pred, cov, plot_type, world):

    plt.figure()
    if world == 'synthetic':
        plt.xlim((-60,60))
        plt.ylim((-40,20))
        A = 4 # uncertainty ellipse inflation
    elif world == 'real':
        plt.xlim((-1,8))
        plt.ylim((-2,1.5))
        A = 1./4

    plt.axes().set_aspect('equal')
    plt.xticks(size = 20)
    plt.yticks(size = 20)

    for i in range(1, len(mean_pred)):
        plt.plot(mean_pred[i][0],mean_pred[i][1], 'r+')
        
    plt.plot(x, y, '-k')
    plt.title('EKF-SLAM estimated path and landmarks position')
    plt.xlabel('x')
    plt.ylabel('y')

    if plot_type == 'include uncertainty':

        last_cov = np.asarray(cov)
        mpred_vec = np.hstack(mean_pred)

        for i in range(3, len(mpred_vec), 3):
            u=mpred_vec[i]                      #x-position of the center
            v=mpred_vec[i+1]                    #y-position of the center
            a, b, t_rot = uncertainty_ellipse(i, last_cov)
            a=A * a       #radius on the x-axis
            b=A * b     #radius on the y-axis
            t_rot = math.pi/2-t_rot                  #rotation angle

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


def make_patch_spines_invisible(ax):
    ax.set_frame_on(True)
    ax.patch.set_visible(False)
    for sp in ax.spines.values():
        sp.set_visible(False)


def cov_time(pose_cov, nr_landmarks_seen, dist):
    
    pose_cov = np.asarray(pose_cov)
    nr_landmarks_seen = np.asarray(nr_landmarks_seen)
    dist = np.asarray(dist)
    k = np.arange(0,len(pose_cov))

    fig, host = plt.subplots()
    fig.subplots_adjust(right=0.75)

    par1 = host.twinx()
    par2 = host.twinx()

    # Offset the right spine of par2.  The ticks and label have already been
    # placed on the right by twinx above.
    par2.spines["right"].set_position(("axes", 1.2))
    # Having been created by twinx, par2 has its frame off, so the line of its
    # detached spine is invisible.  First, activate the frame but make the patch
    # and spines invisible.
    #make_patch_spines_invisible(par2)
    # Second, show the right spine.
    par2.spines["right"].set_visible(True)

    p1, = host.plot(k, pose_cov[:,0,0], 'b-', label='cov_x')
    p2, = host.plot(k, pose_cov[:,1,1], 'r-', label='cov_y')
    p3, = host.plot(k, pose_cov[:,2,2], 'k-', label='cov_heading')

    p4, = par1.plot(k, nr_landmarks_seen, 'm+', label='nr_landmarks_seen')
    p5, = par2.plot(k, dist, 'g-', label='dist')

    par1.set_ylim(0, max(nr_landmarks_seen)*2)

    host.set_xlabel("Iteration, k")
    host.set_ylabel("Covariance")
    par1.set_ylabel("Nr of landmarks seen")
    par2.set_ylabel("Distance")

    host.yaxis.label.set_color(p3.get_color())
    par1.yaxis.label.set_color(p4.get_color())
    par2.yaxis.label.set_color(p5.get_color())

    tkw = dict(size=4, width=1.5)
    host.tick_params(axis='y', colors=p3.get_color(), **tkw)
    par1.tick_params(axis='y', colors=p4.get_color(), **tkw)
    par2.tick_params(axis='y', colors=p5.get_color(), **tkw)
    host.tick_params(axis='x', **tkw)

    lines = [p1, p2, p3]

    host.legend(lines, [l.get_label() for l in lines], loc='upper left')

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
        plt.ylim((-40,20))
        plt.axes().set_aspect('equal')
        plt.xticks(size = 20)
        plt.yticks(size = 20)
    elif world == 'real':
        plt.xlim((-1,8))
        plt.ylim((-2,1.5))
        plt.axes().set_aspect('equal')
        plt.xticks(size = 20)
        plt.yticks(size = 20)

    plt.plot(x_odo,y_odo, '-k')
    plt.title('Odometry estimated path')
    plt.xlabel('x')
    plt.ylabel('y')

    plt.grid(color='lightgray',linestyle='--')
    plt.show(block=False)

def path_and_odometry(x,y, mean_pred, x_odo, y_odo, cov, plot_type, world):

    plt.figure()
    if world == 'synthetic':
        plt.xlim((-60,60))
        plt.ylim((-40,20))
        plt.axes().set_aspect('equal')
        plt.xticks(size = 20)
        plt.yticks(size = 20)
        A = 4 # uncertainty ellipse inflator
    elif world == 'real':
        plt.xlim((-1,8))
        plt.ylim((-2,1.5))
        plt.axes().set_aspect('equal')
        plt.xticks(size = 20)
        plt.yticks(size = 20)
        A = 1./4

    for i in range(1, len(mean_pred)):
        plt.plot(mean_pred[i][0],mean_pred[i][1], 'r+')
     
    plt.plot(x_odo,y_odo, color='yellow')   
    plt.plot(x, y, color='black')
    plt.title('EKF-SLAM estimated path and landmarks position')
    plt.xlabel('x')
    plt.ylabel('y')

    if plot_type == 'include uncertainty':

        last_cov = np.asarray(cov)
        mpred_vec = np.hstack(mean_pred)

        for i in range(3, len(mpred_vec), 3):
            u=mpred_vec[i]                      #x-position of the center
            v=mpred_vec[i+1]                    #y-position of the center
            a, b, t_rot = uncertainty_ellipse(i, last_cov)
            a=A * a       #radius on the x-axis
            b=A * b     #radius on the y-axis
            t_rot = math.pi/2-t_rot                         #rotation angle

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

def uncertainty_ellipse(i, last_cov):

    sigmaxx = last_cov[i][i]
    sigmayy = last_cov[i+1][i+1]
    sigmaxy = last_cov[i][i+1]
    t = 0.5*math.atan(2*sigmaxy/(sigmaxx-sigmayy))

    a = sigmaxx*math.sin(t)**2 + 2*sigmaxy*math.cos(t)*math.sin(t)+sigmayy*math.cos(t)**2
    b = sigmaxx*math.cos(t)**2 - 2*sigmaxy*math.cos(t)*math.sin(t)+sigmayy*math.sin(t)**2

    return a, b, t

def cov_time_lm(lm_cov):
    
    plt.figure()
    lm_cov = np.asarray(lm_cov)
    k = np.arange(0,len(lm_cov))

    plt.plot(k, lm_cov[:,0], 'b-', label='cov_(xi)')
    plt.plot(k, lm_cov[:,1], 'r-', label='cov_(yi)')

    plt.legend(loc="upper right")
    plt.title('First landmark covariance over time')
    plt.show(block=False)