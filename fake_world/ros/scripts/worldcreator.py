#!/usr/bin/env python

def create(worldtype):
    if worldtype=='world1':
        landmark_list=finite_wall(1, 0, 10)
        trajectory=straightline(3, 0, 90*math.pi/180, 0.1, 100)
        print('a')

    if world_type=='world2':
        #landmark_list=self.world2_lm()
        #trajectory_list=self.world2_tr()
        a=1

    return landmark_list, trajectory_list

def finite_wall(x, y, n):
    # ---------------
    # generates and infite wall of landmarks
    # adds a new landmark with the same x-coordinate
    # of the last one but one unit ahead in the
    # y-axis
    # ---------------
    wall=[landmark(x,y,0)]
    i=0
    while i<n:
        x=x
        y=y+1;
        identif=i+1;
        new_landmark=landmark(x,y,identif)
        wall.append(new_landmark)
        i+=1
    return wall

def straightline(x, y, theta, step, n):
    # -------------
    # generates a list of points that corresponds
    # to a straight line trajectory with an assigned step
    # and robot looking up (theta=90deg)
    # -------------
    identif=0
    trajectory=[robot_pose()]
    trajectory[0].x=x
    trajectory[0].y=y
    trajectory[0].theta=theta
    for i in range(n):
        point=robot_pose()
        point.x=trajectory[i].x
        point.y=trajectory[i].y+step
        point.theta=trajectory[i].theta
        trajectory.append(point)
    return trajectory