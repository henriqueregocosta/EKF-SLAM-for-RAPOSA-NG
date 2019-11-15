import rospy
import math

from nav_msgs.msg import Odometry

class OdoInterp(object):

    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''

        rospy.loginfo('Odometry Interpreter Started')
        # subscribe to RaposaNG ARUCO topic
        rospy.Subscriber("fake_odom", Odometry, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        self.position_and_quaternions = []
        self.state = 0
    
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
            
        self.position_and_quaternions = [posx, posy, qx,qy,qz,qw]

        self.state = 1


    def quaternions(self, qx, qy, qz, qw):
        return math.asin(2*(qx*qz - qw*qy)) 


    def odometry_model(self, theta, odometry):
        x_hat, y_hat, qx, qy, qz, qw = self.position_and_quaternions
        
        delta_rot1 = math.atan2(y_hat,x_hat) - theta
        delta_trans = math.sqrt(x_hat*x_hat + y_hat*y_hat)
        
        theta_hat = self.quaternions(qx, qy, qz, qw)
        delta_rot2 = theta_hat - theta - delta_rot1
        return delta_rot1, delta_trans, delta_rot2