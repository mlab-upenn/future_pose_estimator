# future_pose_estimator ROS Package
ROS package that predicts near future pose (up to 1 second) for F1/10th car in front using an April Tag. Written by Christopher Kao for EAS 499 engineering senior thesis.

# Summary
The goal of this ROS package is to reliably predict the near future pose of a F1/10 car that is in front of our F1/10 car. 
Specifically, this package returns the estimated pose of a car 1 second in the future. It provides 10 predictions: for 0.1 second in the future, 0.2 seconds, ..., up to 1.0 seconds in the future. The algorithm used by this package requires a number of pieces of information in order to work correctly.
* lidar scan over the /scan topic
* pre-known map of the world in which the car is driving
* image_raw images coming from a USB webcam facilitated by the ROS usb_cam node
* a set of waypoints that represents the general path that the car is expected to take around the race track map

![two cars in hallway](https://github.com/mlab-upenn/future_pose_estimator/blob/master/photos/L1000398.jpg "Two F1/10 cars in the Levine 2nd floor hallway outside the mLab")
In the image above, notice that there are 2 F1/10 cars. The car behind with the Logitech C910 usb webcam is the car that is running this ROS package. The car in front has an AprilTag of width 10.85cm mounted on its rear.

![close-up on Logitech C910 webcam](https://github.com/mlab-upenn/future_pose_estimator/blob/master/photos/L1000409.jpg "A close-up on the Logitech C910 webcam")
This is a close-up image of the Logitech C910 USB webcam. Although the webcam supports up to 1920x1080 resolution at 30Hz natively, from my tests the highest usable resolution is 640x480 which the apriltag_ros package can process at around 10-12 Hz. A higher resolution such as 1280x720 is unusable for our purposes of near future pose estimation because the apriltag_ros package can only process 720p images at around 4Hz. For 1080p images, the speed is even slower, at around 1-2Hz.

# Instructions for installing and using this package
Assuming that you have ROS Kinetic installed already, do the following:
1. Git clone the mlab-upenn/f110-upenn-course repository (which I was one of the co-contributors for): https://github.com/mlab-upenn/f110-upenn-course 
2. Follow the README instructions in the f110-upenn-course repository to install the required ROS package dependencies.
3. This package requires the apriltag_ros repository which is located here: https://github.com/AprilRobotics/apriltag_ros. Git clone this into the /algorithms folder which you will see inside the f110-upenn-course repository which you have cloned.
4. Then, git clone *this repository* into the /algorithms folder. You should now see a folder called /future_pose_estimator. 
5. You will then need to sudo apt-get a few packages that this package depends on for usb cam images.

sudo apt-get install ros-kinetic-usb-cam ros-kinetic-camera-calibration

At this point, navigate to the root of your workspace (ws) ROS directory, and type catkin_make. This may take a few minutes to build. If you see any errors, there may be other ROS packages that you need to install. The error messages will give you hints as to what you need to install. The command to install will be of the form sudo apt-get install ros-kinetic-[insert package name with hyphens]

# Description of package files


# Details on hardware setup on F1/10 car
(Here can insert an image of the car setup, from one of the images from yesterday or from before)


# Description of algorithm






