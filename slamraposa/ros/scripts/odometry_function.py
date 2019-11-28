import rospy
import math
from nav_msgs.msg import Odometry
import tf

class OdoInterp(object):

    def __init__(self, queue_name, R):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''

        rospy.loginfo('Odometry Interpreter Started')
        # subscribe to RaposaNG ARUCO topic
        # raposa topic: /raposang/odometry fake_world topic: fake_odom
        self.subs = rospy.Subscriber("/raposang/odometry_pose", Odometry, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        
        self.q = queue_name
        self.R = R
        self.last_odom = [0,0,0]
    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a nav_msgs/Odometry msg is received on the
        topic: /fake_odom
        '''
        
        posx = msg.pose.pose.position.x
        posy = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _,_,theta = tf.transformations.euler_from_quaternion((qx, qy, qz, qw))

        self.q.put(['odo', self.odometry_model(posx, posy, theta), self.R])
        self.last_odom = [posx, posy, theta]

    def odometry_model(self, posx, posy, theta):

        delta_rot1 = math.atan2(posy - self.last_odom[1], posx - self.last_odom[0]) - self.last_odom[2]
        delta_trans = math.sqrt((posy - self.last_odom[1])**2 + (posx - self.last_odom[0])**2)
        print(theta)
        print(self.last_odom[2])
        print(delta_rot1)
        delta_rot2 = theta - self.last_odom[2] - delta_rot1

        if delta_rot1 < -math.pi:
            delta_rot1 += 2*math.pi
        if delta_rot1 > math.pi:
            delta_rot1 += -2*math.pi

        if delta_rot2 < -math.pi:
            delta_rot2 += 2*math.pi
        if delta_rot2 > math.pi:
            delta_rot2 += -2*math.pi

        return delta_rot1, delta_trans, delta_rot2