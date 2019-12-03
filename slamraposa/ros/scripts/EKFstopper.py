import rospy
from std_msgs.msg import String

class shutdown(object):

    def __init__(self, queue_name):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        rospy.loginfo('EKF stopper Started')
        # subscribe to RaposaNG ARUCO topic: /aruco_marker_publisher/markers
        # fake_world topic: fake_obs
        self.subs = rospy.Subscriber("killmessage", String, self.KillCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls

        self.q = queue_name
    
    def KillCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''
        
        self.q.put(['end', 'EKF should stop now', 0])