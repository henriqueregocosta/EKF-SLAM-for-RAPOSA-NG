import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class ObsInterp(object):

    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        rospy.loginfo('Observations Interperter Started')
        # subscribe to RaposaNG ARUCO topic
        rospy.Subscriber("fake_obs", MarkerArray, self.ARUCOCallback)
        print('Subscribe')
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        self.markersisee = []
        self.state = 0 #

    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''

        self.state = 1
        N = len(msg.markers)
        rospy.loginfo(str(N))

        for i in range(N):
            ox = msg.markers[i].pose.position.x
            oy = msg.markers[i].pose.position.y
            oid = msg.markers[i].id
            update = [ox, oy, oid]

            self.markersisee.append(update) # the marker is added to the vector