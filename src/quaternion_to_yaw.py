#!/usr/bin/env python

import math
import rospy
import tf
import csv
import os
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# The purpose of this Python node is mainly for the detect_apriltags.launch file. This node outputs the xyz position of the singular AprilTag as well as the yaw of the AprilTag, in degrees. This is useful in debugging, particularly when measuring the distances and angles of AprilTags in real life to do a sanity check that the measurements look correct.

def callback(data):
    if len(data.detections) == 0:
        print("No AprilTag detected")
    else:
	    pose = data.detections[0].pose.pose.pose

	    orientation = pose.orientation
	    position = pose.position

	    quaternion = (
		    orientation.x,
		    orientation.y,
		    orientation.z,
		    orientation.w)
	    euler = tf.transformations.euler_from_quaternion(quaternion)
	    yaw = euler[1] # in radians, where negative is yawed left and positive is yawed right.
	    yaw_in_degrees = yaw * 180.0 / 3.14159265

	    print(position)
	    print("yaw in degrees:", yaw_in_degrees)
	    print("")


if __name__ == '__main__':
	print("Converting quaternion to yaw angle")
	rospy.init_node('quaternion_to_yaw', anonymous = True)
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	rospy.spin()
