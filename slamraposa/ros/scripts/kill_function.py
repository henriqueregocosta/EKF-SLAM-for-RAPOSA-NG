import rospy
from std_msgs.msg import String
import Queue



class KillInterp(object):

    def __init__(self, queue_name):
        rospy.loginfo('Kill Interpreter Started')
        # subscribe to RaposaNG ARUCO topic
        self.subs = rospy.Subscriber("killmessage", String, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        self.q = queue_name

    
    def ARUCOCallback(self, msg):
        self.q.put(['kill'])