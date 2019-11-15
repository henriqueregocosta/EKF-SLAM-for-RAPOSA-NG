#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy

def plot_world(landmark_list, trajectory, observations, world):

    xl=[]
    yl=[]

    n = len(landmark_list)
    for i in range(n):
        xl.append(landmark_list[i].x)
        yl.append(landmark_list[i].y)

    n = len(trajectory)
    x = []
    y = []
    for i in range(n):
        x.append(trajectory[i].x)
        y.append(trajectory[i].y)


    xxl=np.asarray(xl)
    yyl=np.asarray(yl)
    xx=np.asarray(x)
    yy=np.asarray(y)

    

    if world=='world1':
        plt.ylim((0,15))
        plt.xlim((0,5))
    if world=='world2':
        plt.ylim((-5,5))
        plt.xlim((-5,5))

    if world=='world3':
        plt.ylim((-5,5))
        plt.xlim((-5,5))

    if world=='worl':
        plt.ylim((0,10))
        plt.xlim((-1,9))

    

    plt.plot(xx, yy, '-b', xxl, yyl, 'r^')
    plt.ylim(-1,9)
    plt.xlim(-1,9)
    plt.ylabel('y')
    plt.xlabel('x')
    plt.show()

def plot_SLAM(x, y, mean):

    xx=np.asarray(x)
    yy=np.asarray(y)

    plt.plot(xx, yy, '-b')

    plt.ylim((-1,9))
    plt.xlim((-1,9))

    plt.show()

