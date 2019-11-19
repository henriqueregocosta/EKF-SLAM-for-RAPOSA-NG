import rospy
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class ObsInterp(object):

    def __init__(self, queue_name, Q, R):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''

        rospy.loginfo('Observations Interpreter Started')
        # subscribe to RaposaNG ARUCO topic
        rospy.Subscriber("fake_obs", MarkerArray, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        
        self.markers_I_see = []
        self.q = queue_name
        self.Q = Q
        self.R = R

    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''

        N = len(msg.markers)

        for i in range(N):
            ox = msg.markers[i].pose.position.x
            oy = msg.markers[i].pose.position.y
            oid = msg.markers[i].id
            update = [ox, oy, oid]

            self.markers_I_see.append(update) # the marker is added to the vector

        self.q.put(['obs', self.markers_I_see, self.Q])