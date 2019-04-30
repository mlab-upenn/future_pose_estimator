#!/usr/bin/env python

import math
import rospy
import tf
from apriltags2_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose

# Create a transform broadcaster
br = tf.TransformBroadcaster()

# Global variable for prev_pose
prev_pose = Pose()
has_prev_pose = True # flag so that we know whether there was a previous pose
HERTZ = 12 # apriltags2_ros is giving us pose estimates at around 12 times per second for 640 x 480 resolution.

# This is the linear model approach, which looks at the leading car's current orientation (yaw), position, and velocity. It projects it forward at the current velocity in a linear fashion using sine and cosine of the yaw angle.

# For simplicity, let's just assume for now that there is only one id. And that that id is id=0 from 36h11 April Tag class.

# z is away from the camera
# x is to the right of the camera, from camera's perspective
# y is down

def callback(data):
	global prev_pose
	global has_prev_pose

	num_detections = len(data.detections)
	
	# For now, we will just assume that we are only looking for one id, particularly with id=0. 
	if num_detections > 0:
		pose = data.detections[0].pose.pose.pose
		if has_prev_pose: # only go through calculations if had a valid previous pose
			position = pose.position
			orientation = pose.orientation

            # Linear model approach. Get the relative speed of the car and the orientation, and use that to predict its future pose.
            # Orientation is given as x, y, z, w. How do we convert that into yaw? Don't care about pitch and roll.
			delta_x = position.x - prev_pose.position.x # expressed in meters
			delta_z = position.z - prev_pose.position.z

			velocity = math.sqrt(delta_x * delta_x + delta_z * delta_z) * HERTZ # meters per second
			if delta_z < 0:
				velocity = velocity * -1
			print("velocity: ", velocity)

			quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			yaw = euler[1] # in radians, where negative is yawed left and positive is yawed right
			yaw_in_degrees = yaw * 180.0 / 3.14159265
			print("yaw in degrees: ", yaw_in_degrees)

			predicted_x = position.x + velocity * math.sin(yaw)
			predicted_z = position.z + velocity * math.cos(yaw)

			# Send transform broadcast
			br.sendTransform((predicted_x, position.y, predicted_z),
					 (orientation.x, orientation.y, orientation.z, orientation.w),
					  rospy.Time.now(),
					  "future_pose_estimation",
				 	  "camera")
			
		# if didn't have a valid previous pose, then don't do calculations. But still save the previous pose.

		# Update previous pose with this current pose
		prev_pose = pose
		has_prev_pose = True
		

	else:
		print "No tag detected, so not publishing anything."
		has_prev_pose = False
	

if __name__ == '__main__':
	print("Future pose estimator node started...")
	rospy.init_node('future_pose_estimator',anonymous = True)
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	rospy.spin()
