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

#home = expanduser('~')
#file = open(strftime(home+'/rcws/logs/wp-%Y-%m-%d-%H-%M-%S',gmtime())+'.csv', 'w')
file = open("predicted_poses.csv", "w")

def save_waypoint(data):
    for marker in data.markers:
        x = marker.pose.position.x
        y = marker.pose.position.y

        file.write('%f, %f, %f\n' % (x, y, 0))

#    quaternion = np.array([data.pose.orientation.x, 
#                           data.pose.orientation.y, 
#                           data.pose.orientation.z, 
#                           data.pose.orientation.w])

#    euler = tf.transformations.euler_from_quaternion(quaternion)
#    speed = LA.norm(np.array([data.twist.twist.linear.x, 
#                              data.twist.twist.linear.y, 
#                              data.twist.twist.linear.z]),2)

#    file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
#                                     data.pose.pose.position.y,
#                                     euler[2],
#                                     speed))
    

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
