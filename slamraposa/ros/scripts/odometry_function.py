import rospy
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