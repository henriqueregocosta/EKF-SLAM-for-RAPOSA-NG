#!/usr/bin/env python
import slamraposa_ros.slamraposa_node

import numpy as np
import math
import rospy
import observations_function as obs
import odometry_function as odo
import EKF_SLAM


def main():
    rospy.loginfo('Hello from main1')
    operation = EKF_SLAM.SLAM()

    rospy.loginfo('Hello from main2')

    observations = obs.ObsInterp()
    odometry = odo.OdoInterp()

    while not rospy.is_shutdown():
        # atualiza-las em funcao do tempo de espera, da velocidade do robot, etc...
        rospy.loginfo('Hello from main while')
        Q = np.identity(3)
        R = np.identity(3)
        operation.EKF(odometry, observations, Q, R)

rospy.loginfo('Hello from main1')
if __name__ == '__main__':
    slamraposa_ros.slamraposa_node.main()
