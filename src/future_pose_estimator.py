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
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

# Create a transform broadcaster
broadcaster = tf.TransformBroadcaster()

# Create Marker Array publisher
publisher = rospy.Publisher("/future_pose", MarkerArray, queue_size="1")

# Global variable for prev_pose
prev_pose = Pose()
has_prev_pose = True # flag so that we know whether there was a previous pose
HERTZ = 4 # apriltags2_ros is giving us pose estimates at around 4 times per second for 1280 x 720 resolution.
LOOKAHEAD_DISTANCE = 2.0 # meters

# import waypoints.csv into a list
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
	path_points = [tuple(line) for line in csv.reader(f)]

# For simplicity, let's just assume for now that there is only one id. And that that id is id=0 from 36h11 April Tag class.

# z is away from the camera
# x is to the right of the camera, from camera's perspective
# y is down

# 6. We can then write the code so that it does a weighting between what the previous linear strategy for deriving future pose (from velocity and pose), weighted with the future pose from pure pursuit. 

# Function that takes in the pose of April Tag id_0 from the map to id_0 transform. 
# Outputs the steering angle and velocity that the car in front should take to get to the next waypoint.
# The callback function will call pure_pursuit incrementally on 1/10 second increments. 
# So it will predict where the car will be in 0.10 seconds using a linear model with vcos(theta) and vsin(theta), then call pure_pursuit again on that new pose for 0.20 seconds, then 0.30 seconds, etc. 
# input variable data contains orientation and position information, with respect to the map
def pure_pursuit(x, y, yaw): # assume for now that we know the pose of the car in front already
	# 1. Determine the current location of the vehicle (we are subscribed to vesc/odom)
	# print " "
	# print "x:", x
#	print "y:", y
#	print "yaw:", yaw

	# 2. Find the path point closest to the vehicle
	# Index from end of the list and say, is this point close enough. And keep going
	# until one of those waypoints that is say, less than, 2 meters from you.
	# Trick: Once initialized, can just look at indices +- 10 from where current index is.
	# Vehicle should steer towards closest point one lookahead distance from its current location.
	for point in reversed(path_points):
		# Calculate distance between (x,y) and (point_x, point_y)
		point_x = float(point[0])
		point_y = float(point[1])
		dist = np.sqrt((x-point_x)**2 + (y-point_y)**2)
		if dist <= LOOKAHEAD_DISTANCE:
			break
		
	# At this point we should have found the goal point.
	
	# 3. Find the goal point.
	goal_point = point
#	print "goal point:", point_x, point_y
	# 4. Transform the goal point to vehicle coordinates. 
	# Vehicle local coordinates probably need to be rotated by quaternion orientation.
#	print "dist:", dist

	# theta should be minimum of yaw and math.pi/2 - yaw
	theta = yaw # theta is car's heading relative to world origin x axis in radians (initially is 0)
	# beta = abs(math.atan2((point_x - x), (point_y - y))) # direction in radians to goal point in world coordinates
	beta = math.atan2((point_x - x), (point_y - y))
	gamma = math.pi / 2 - theta - beta # direction in radians to goal point in car's local coordinates where positive gamma is left and negative gamma is right

#	print "theta:", theta
#	print "beta:", beta
#	print "gamma:", gamma

	point_x_wrt_car = dist * math.sin(gamma) * -1.0
	point_y_wrt_car = dist * math.cos(gamma)

#	print "point x wrt car:", point_x_wrt_car
#	print "point y wrt car:", point_y_wrt_car

	# 5. Calculate the curvature.
	# The curvature is transformed into steering wheel angle by the vehicle on board controller.
	# curvature = 1/r = 2x/l^2
	angle = 2 * point_x_wrt_car / dist**2
	angle = -1.0 * angle # need to flip signage since negative angle goes right
#	print "angle:", angle

	# 6. Update the vehicle's position. This is basically fetching the next subscription /vesc/odom. [DONE]

#	print "Angle in Degrees", angle*180/np.pi

	if angle > 0.5236: # 0.5236 radians = 30 degrees
		# print "Left steering angle too large!"
		angle = 0.5236
	if angle < -0.5236:
		# print "Right steering angle too large!"
		angle = -0.5236

	return angle # this is the angle in radians that car in front would take according to pure pursuit to follow the set of waypoints

def callback(data):
	global prev_pose
	global has_prev_pose
	global listener

	num_detections = len(data.detections)
	
	# For now, we will just assume that we are only looking for one id, particularly with id=0. 
	if num_detections > 0:
		pose = data.detections[0].pose.pose.pose

		# First get the transform between map and other_base_link.
		
		listener.waitForTransform("/map", "/other_base_link", rospy.Time(), rospy.Duration(1.0))
		(position, orientation) = listener.lookupTransform("/map", "/other_base_link", rospy.Time(0))
		# Convert tf to x, y, and yaw (heading)			
		quaternion = (
			orientation[0],
			orientation[1],
			orientation[2],
			orientation[3]
		)
		euler = euler_from_quaternion(quaternion)
		current_x = position[0]
		current_y = position[1]
		current_heading = euler[2] # yaw relative to the map

		# Then manually calculate the predicted_x and predicted_y of the car in the map frame, 1/10th of a second into the future. 
		
		velocity = 1.0 # hard code in some placeholder number for now
		# TODO: Velocity of other car can be estimated by current car velocity + relative velocity.
		# TODO: But for now, we can assume that the car is at constant 1.0m/s velocity for simplicity.
		timestep = 0.1 # seconds
		wheel_base = 0.325 # 32.5 cm
		marker_array = MarkerArray()
		alpha = 0.7 # used for weighting between pure pursuit and linear model

		for i in range(10): # TODO: Change this to 10 later
			# Then call pure_pursuit function on that pose.
			steering_angle = pure_pursuit(current_x, current_y, current_heading)

			angular_velocity = velocity / wheel_base * math.tan(steering_angle) # rad/second
			angular_change = timestep * angular_velocity
			predicted_heading_pure_pursuit = current_heading + angular_change
			
			# TODO: Implement the weighted average. 
			predicted_heading_linear_model = current_heading # linear model says current heading doesn't change
			weighting = alpha**i # raised to the i between 0 - 9
			predicted_heading_weighted_averaged = weighting * predicted_heading_linear_model + (1 - weighting) * predicted_heading_pure_pursuit
			# Can get the predicted_x and predicted_y using linear model for velocity * timestep
			# We know the current_heading of the car. Hence should be able to linearly project out
			#   where the car will be amount of time from now.

			predicted_x = current_x + velocity * timestep * math.cos(predicted_heading_weighted_averaged)
			predicted_y = current_y + velocity * timestep * math.sin(predicted_heading_weighted_averaged) # TODO :Test this out after going to POD to see if it works

#			print("timestep:", i)
#			print("predicted_x:", predicted_x)
#			print("predicted_y:", predicted_y)
			# print("i:", i)
			# print("weighting:", weighting)
			# print("predicted_heading_pure_pursuit:", predicted_heading_pure_pursuit)
			# print("predicted_heading_linear_model:", predicted_heading_linear_model)
			# print("predicted_heading_weighted_averaged:", predicted_heading_weighted_averaged)
			# print ""

			# Add (predicted_x, predicted_y) to marker_array
			marker = Marker()
			marker.header.frame_id = "/map"
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.scale.x = 0.1
		        marker.scale.y = 0.1
		        marker.scale.z = 0.1
		        marker.color.a = 1.0
		        marker.color.r = 0.0
		        marker.color.g = 1.0
		        marker.color.b = 0.0
			marker.pose.position.x = predicted_x
			marker.pose.position.y = predicted_y
			marker.pose.position.z = 0
			marker_array.markers.append(marker)

			# Lastly set current_x = predicted_x
			current_x = predicted_x
			current_y = predicted_y
			current_heading = predicted_heading_weighted_averaged
			
		# Renumber the marker IDs
		id = 0
		for m in marker_array.markers:
			m.id = id
			id += 1
		publisher.publish(marker_array)
		# TODO: IMPORTANT: Next step is to be able to just plot the set of 10 future pose estimates for the car if running pure pursuit incrementally. And output these as maybe a set of points with orientations in Rviz.

		# These are calculations for linear model, so with just the velocity and yaw
		if has_prev_pose: # only go through calculations if had a valid previous pose		
			position = pose.position
			orientation = pose.orientation

			# Approach #1. Linear (x, z) extrapolation approach is here.
			delta_x = position.x - prev_pose.position.x # expressed in meters
			delta_z = position.z - prev_pose.position.z
			# Ignore y (up and down in camera's perspective) since we assume F1/10 cars are racing in a 2D plane
			velocity = math.sqrt(delta_x * delta_x + delta_z * delta_z) * HERTZ # meters per second
			if delta_z < 0:
				velocity = velocity * -1

			# Approach #2. Get the relative speed of the car and the orientation, and use that to predict 
			# its future pose. 
			# Orientation is given as x, y, z, w. How do we convert that into yaw? Don't care about pitch and 			  roll.
			quaternion = (
				orientation.x,
				orientation.y,
				orientation.z,
				orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			yaw = euler[1] # in radians, where negative is yawed left and positive is yawed right.
			yaw_in_degrees = yaw * 180.0 / 3.14159265

			predicted_x = position.x + velocity * math.sin(yaw)
			predicted_z = position.z + velocity * math.cos(yaw)

			# Send transform broadcast
			# broadcaster.sendTransform((predicted_x, position.y, predicted_z),
					 #(orientation.x, orientation.y, orientation.z, orientation.w),
					  #rospy.Time.now(),
					  #"future_pose_estimation",
				 	  #"camera")
			
		# if didn't have a valid previous pose, then don't do calculations. But still save the previous pose.

		# Update previous pose with this current pose
		prev_pose = pose
		has_prev_pose = True
		

	else:
		print "No tag detected, so not publishing anything."
		has_prev_pose = False
	

if __name__ == '__main__':
	print("Future pose estimator node started...")
	rospy.init_node('future_pose_estimator', anonymous = True)
	# Create a transform listener
	global listener
	listener = tf.TransformListener()
	
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	
	rospy.spin()
