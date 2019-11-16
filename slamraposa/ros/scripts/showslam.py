#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rospy

def plot_SLAM(x, y, mean):

    xx=np.asarray(x)
    yy=np.asarray(y)

    plt.plot(xx, yy, '-b')

    for i in range(1, len(mean)):
        plt.plot(mean[i][0],mean[i][1], 'r^')

    plt.ylim((-1,4))
    plt.xlim((-1,4))

    plt.show()

