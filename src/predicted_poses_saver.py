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

# This node subscribes to the /future_pose topic which is published by future_pose_estimator.py and
# saves the (x, y) coordinates into a file called predicted_poses.csv.

file = open("predicted_poses.csv", "w")

def save_waypoint(data):
    for marker in data.markers:
        x = marker.pose.position.x
        y = marker.pose.position.y

        file.write('%f, %f, %f\n' % (x, y, 0))

def shutdown():
    file.close()
    print('Goodbye')
 
def listener():
    rospy.init_node('predicted_poses_saver', anonymous=True)
    rospy.Subscriber('/future_pose', MarkerArray, save_waypoint)
    rospy.spin()

if __name__ == '__main__':
    atexit.register(shutdown)
    print('Saving predicted poses as waypoints...')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
