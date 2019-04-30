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

HERTZ = 12 # apriltags2_ros is giving us pose estimates at around 12 times per second for 640 x 480 resolution.
LOOKAHEAD_DISTANCE = 2.0 # meters

# import waypoints.csv into a list
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
with open(filename) as f:
	path_points = [tuple(line) for line in csv.reader(f)]

# For simplicity, let's just assume for now that there is only one id. And that that id is id=0 from 36h11 April Tag class.
# This is a valid assumption because we are assuming only two cars, and hence the car in front would have one AprilTag.

# z is away from the camera
# x is to the right of the camera, from camera's perspective
# y is down

# Function that takes in position of the car with the AprilTag with respected to the map, as well as its yaw with respect to the map.
# Outputs the steering angle and velocity that the car in front should take to get to the next waypoint.
# The callback function will call pure_pursuit incrementally on 1/10 second increments. 
# So it will predict where the car will be in 0.10 seconds using a linear model with vcos(theta) and vsin(theta), then call pure_pursuit again on that new pose for 0.20 seconds, then 0.30 seconds, etc.
def pure_pursuit(x, y, yaw): # assume for now that we know the pose of the car in front already
	# Find the path point closest to the vehicle
	# Index from end of the list and say, is this point close enough. And keep going
	# until one of those waypoints that is say, less than, 2 meters from you.
	# Vehicle should steer towards closest point one lookahead distance from its current location.
	for point in reversed(path_points):
		# Calculate distance between (x,y) and (point_x, point_y)
		point_x = float(point[0])
		point_y = float(point[1])
		dist = np.sqrt((x-point_x)**2 + (y-point_y)**2)
		if dist <= LOOKAHEAD_DISTANCE:
			break
		
	# At this point we should have found the goal point.
	goal_point = point
    
	# Transform the goal point to vehicle coordinates.
	# theta should be minimum of yaw and math.pi/2 - yaw
	theta = yaw # theta is car's heading relative to world origin x axis in radians (initially is 0)
	# beta = abs(math.atan2((point_x - x), (point_y - y))) # direction in radians to goal point in world coordinates
	beta = math.atan2((point_x - x), (point_y - y))
	gamma = math.pi / 2 - theta - beta # direction in radians to goal point in car's local coordinates where positive gamma is left and negative gamma is right
	point_x_wrt_car = dist * math.sin(gamma) * -1.0
	point_y_wrt_car = dist * math.cos(gamma)

	# Calculate the curvature needed for the car to reach the goal point.
	# The curvature is transformed into steering wheel angle by the vehicle on board controller.
	# curvature = 1/r = 2x/l^2
	angle = 2 * point_x_wrt_car / dist**2
	angle = -1.0 * angle # need to flip signage since negative angle goes right

	# Update the vehicle's position. This is basically fetching the next subscription /vesc/odom. [DONE]

	if angle > 0.5236: # 0.5236 radians = 30 degrees
		# print "Left steering angle too large!"
		angle = 0.5236
	if angle < -0.5236:
		# print "Right steering angle too large!"
		angle = -0.5236

	return angle # this is the angle in radians that car in front would take according to pure pursuit to follow the set of waypoints

def callback(data):
	global listener

    num_detections = len(data.detections) # number of AprilTag detections in the image
	
    # Do the following if an AprilTag is detected.
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
		#       But for now, we can assume that the car is at constant 1.0m/s velocity for simplicity.
		timestep = 0.1 # seconds
		wheel_base = 0.325 # 32.5 cm
		marker_array = MarkerArray()
		alpha = 0.7 # used for weighting between pure pursuit and linear model

		for i in range(10):
			# Then call pure_pursuit function on that pose.
			steering_angle = pure_pursuit(current_x, current_y, current_heading)

			angular_velocity = velocity / wheel_base * math.tan(steering_angle) # rad/second
			angular_change = timestep * angular_velocity
			predicted_heading_pure_pursuit = current_heading + angular_change
			
			predicted_heading_linear_model = current_heading # linear model says current heading doesn't change
			weighting = alpha**i # raised to the i between 0 - 9
            # Computes the angle that the car would be likely to take at this multiple of 0.1 second step in time.
			predicted_heading_weighted_averaged = weighting * predicted_heading_linear_model + (1 - weighting) * predicted_heading_pure_pursuit

			predicted_x = current_x + velocity * timestep * math.cos(predicted_heading_weighted_averaged)
			predicted_y = current_y + velocity * timestep * math.sin(predicted_heading_weighted_averaged)

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
			
		# Renumber the marker IDs (I think this allows it to publish correctly, not even sure if it is necessary to renumber)
		id = 0
		for m in marker_array.markers:
			m.id = id
			id += 1
		publisher.publish(marker_array)
		
	else:
		print "No tag detected, so not publishing anything."
	

if __name__ == '__main__':
	print("Future pose estimator node started...")
	rospy.init_node('future_pose_estimator', anonymous = True)
	# Create a transform listener
	global listener
	listener = tf.TransformListener()
	
	rospy.Subscriber("/tag_detections", AprilTagDetectionArray, callback)
	
	rospy.spin()
