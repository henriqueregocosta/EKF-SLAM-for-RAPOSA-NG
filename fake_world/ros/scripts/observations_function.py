class ObsInterp(object):

    def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''

        rospy.loginfo('Observations Interperter Started')
        # subscribe to RaposaNG ARUCO topic
        rospy.Subscriber("fake_obs", MarkerArray, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls
        self.markersisee = [None, None, None]

    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''
        observed_features = [None, None, None]
        N = len(msg.markers)
        rospy.loginfo(str(N))

        for i in range(N):
            ox = msg.markers[i].pose.position.x
            oy = msg.markers[i].pose.position.y
            oid = msg.markers[i].id
            print('oid')
            print(oid)
            update = [ox, oy, oid]

            if observed_features == [None, None, None]: #if it is the first marker ever observed
                observed_features = update       #the vector is the first marker
            else:
                observed_features.append(update) #the marker is added to the vector

        self.markersisee = observed_features
        rospy.loginfo('Observed features:')
        rospy.loginfo(str(observed_features))