#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# This node subscribes to the transform from /map to /other_base_link and
# saves the (x, y) coordinates into a file called predicted_poses.csv.
# This position is considered the "actual" position of the car in front.

file = open("front_car_poses.csv", "w")

def shutdown():
    file.close()
    print('Goodbye')

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving predicted poses as waypoints...')

    rospy.init_node('front_car_pose_saver', anonymous=True)
    
    # Create a transform listener
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (position, orientation) = listener.lookupTransform("/map", "/other_base_link", rospy.Time(0))
        except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(position)
        print("")
        		
        x = position[0]
        y = position[1]
        file.write('%f, %f, %f\n' % (x, y, 0))
        
        rate.sleep()
