import rospy
from visualization_msgs.msg import MarkerArray

class ObsInterp(object):

    def __init__(self, queue_name, Q):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        rospy.loginfo('Observations Interpreter Started')
        # subscribe to RaposaNG ARUCO topic: aruco_marker_publisher/markers
        # fake_world topic: fake_obs
        self.subs = rospy.Subscriber("fake_obs", MarkerArray, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls

        self.q = queue_name
        self.Q = Q

    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''
        markers_I_see = []
        N = len(msg.markers)

        for i in range(N):
            ox = msg.markers[i].pose.position.x
            oy = msg.markers[i].pose.position.y
            oid = msg.markers[i].id
            update = [ox, oy, oid]

            markers_I_see.append(update) # the marker is added to the vector
        
        self.q.put(['obs', markers_I_see, self.Q])