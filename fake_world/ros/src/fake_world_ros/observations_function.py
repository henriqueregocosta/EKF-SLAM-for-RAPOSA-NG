
# Não sei como é que este import funciona, verificar com o Henrique

from Marker.msg import Marker

class ObsInterp(object):

	def __init__(self):
        '''
        Class constructor: will get executed at the moment
        of object creation
        '''
        observed_features = [0 0 0]

        rospy.loginfo('Observations Interperter Started')
        # subscribe to RaposaNG ARUCO topic
        rospy.Subscriber("fake_obs", Marker, self.ARUCOCallback)
        # define member variable and initialize with a big value
        # it will store the distance from the robot to the walls

    
    def ARUCOCallback(self, msg):
        '''
        This function gets executed everytime a ARUCO Marker msg is received on the
        topic: /fake_obs
        '''
        ox = msg.pose.position.x
        oy = msg.pose.position.y
        oid = msg.id
        update = [ox oy oid]
        first = 0

        if len(observed_features) == [0 0 0] #if it is the first marker
        	observed_features = update		 #the vector is the first marker
    	else
    		observed_features = mean_pred.append(update) #the marker is added to the vector

