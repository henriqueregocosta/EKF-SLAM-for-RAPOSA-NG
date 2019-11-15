#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

def plot_world(landmark_list, trajectory, observations, world):

    n = len(landmark_list)
    for i in range(n):
        plt.plot(landmark_list[i].x, landmark_list[i].y, 'r^')
        plt.ylabel('y')
        plt.xlabel('x')

    n = len(trajectory)
    x = []
    y = []
    for i in range(n):
        x.append(trajectory[i].x)
        y.append(trajectory[i].y)

    xx=np.asarray(x)
    yy=np.asarray(y)
    plt.plot(xx, yy, '-b')

    if world=='world1':
        plt.ylim((0,15))
        plt.xlim((0,5))
    if world=='world2':
        plt.ylim((-5,5))
        plt.xlim((-5,5))

    if world=='world3':
        plt.ylim((-5,5))
        plt.xlim((-5,5))

    plt.ylim(-1,9)
    plt.xlim(-1,9)

    plt.show()
