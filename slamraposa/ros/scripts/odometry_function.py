import rospy
from nav_msgs.msg import Odometry

class OdoInterp(object):

    def __init__(self, queue_name, R):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''

        rospy.loginfo('Odometry Interpreter Started')
        # subscribe to RaposaNG ARUCO topic
        self.subs = rospy.Subscriber("fake_odom", Odometry, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        
        self.q = queue_name
        self.R = R
    
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


        self.q.put(['odo', [posx, posy, qx,qy,qz,qw], self.R])