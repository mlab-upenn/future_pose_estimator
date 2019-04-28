# future_pose_estimator ROS Package
ROS package that predicts near future pose (up to 1 second) for F1/10th car in front using an April Tag. Written by Christopher Kao for EAS 499 engineering senior thesis in Spring 2019. Advised by Professor Rahul Mangharam and Professor Camillo J. Taylor.

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

![bird's eye view of F1/10 car components](https://github.com/mlab-upenn/future_pose_estimator/blob/master/photos/L1000412.jpg "bird's eye view of F1/10 car components")
This image above showcases the hardware on the F1/10 car. Going from left to right, the green board is the power board which regulates power from the Traxxas LiPo battery to the Jetson TX2 computer (in blue), the Hokuyo 30LX lidar (black and orange on the right), and VESC (occluded since it is "inside" the car). There are also 2 USB 3.0 hubs which allow for accessories which communicate with the VESC, Hokuyo lidar, Logitech C910 camera, joystick, and keyboard and mouse when plugged in. Also note that on the blue board on the left there is a 64GB microSD card which is important for saving bag files. A simple 3 minute bag file takes up over 4GB because of all the images that are saved, even though they are at a relatively low resolution of 640x480.

# Instructions for installing and using this package
Assuming that you have ROS Kinetic installed already, do the following:
1. Git clone the mlab-upenn/f110-upenn-course repository (which I was one of the co-contributors for): https://github.com/mlab-upenn/f110-upenn-course 
2. Follow the README instructions in the f110-upenn-course repository to install the required ROS package dependencies.
3. This repository requires the particle filter from MIT-racecar, instructions to install are here: https://github.com/mlab-upenn/f110-upenn-course/tree/master/algorithms/particle_filter. 
4. This package requires the apriltag_ros repository which is located here: https://github.com/AprilRobotics/apriltag_ros. Git clone this into the /algorithms folder which you will see inside the f110-upenn-course repository which you have cloned.
5. Then, git clone *this repository* into the /algorithms folder. You should now see a folder called /future_pose_estimator. 
6. You will then need to sudo apt-get a few packages that this package depends on for usb cam images.

sudo apt-get install ros-kinetic-usb-cam ros-kinetic-camera-calibration

At this point, navigate to the root of your workspace (ws) ROS directory, and type catkin_make. This may take a few minutes to build. If you see any errors, there may be other ROS packages that you need to install. The error messages will give you hints as to what you need to install. The command to install will be of the form sudo apt-get install ros-kinetic-[insert package name with hyphens]

You will also want to print your own AprilTag. You can do this with the id0 pdf AprilTag found in the /apriltag-printout folder in this repository.

# Description of package files
These are the descriptions of the folders in this future_pose_predictor package:

/config
- Contains camera calibrations for the Logitech C910 webcam at various resolutions. The one that is used in the launch files of this package is the logitech_c910_calibration_640x480.yaml file. 
- Also contains rviz_custom.rviz, which is a custom Rviz configuration which showcases the map, laser scans, image raw, AprilTag processed image raw, and much more.
- settings.yaml is a modified version of a file from the apriltags2_ros package. This is an input parameter to the apriltags2_ros package in the launch file. Here I have told apriltags2_ros to only look for tag36h11 family, which is the default family for AprilTag. I have also asked apriltags2_ros to publish the transform from the camera to the tag. I have not changed the remaining parameters.
- tags.yaml contains the list of April Tags that should be recognized. Here I have only specified that one tag be identified, which is tag with id 0. I made this conscious decision because I am assuming a race with only 2 cars, and that we only need to detect the specific tag on the back of the car in front. Note that this file also needs to specify the size of the tag, which is 0.1085 meters. If you print your AprilTag at different sizes, you can. Just make sure to update this file. This file is specified as a parameter for launching apriltags2_ros in the launch files in the /launch folder.

/launch
- detect_apriltags.launch opens Rviz and shows the detected AprilTag, as well as the transform from the camera to the AprilTag.
![screenshot of detect_apriltags.launch](https://github.com/mlab-upenn/future_pose_estimator/blob/master/photos/detect-apriltag-launch.png "Screenshot of detect_apriltags.launch")
This screenshot above represents what you would see when running "roslaunch future_pose_predictor detect_apriltags.launch" in the terminal. On the right, you see an Rviz window. The large window on the right represents the transform from the camera to the id_0 AprilTag. On the left, notice Raw Image window (the image on top) which is taken in the mLab and shows the id_0 AprilTag on the left side. Below that, the ProcessedImage is the same image but with highlighted boxes around the AprilTag. On the far left of the screenshot, notice a stream of xyz AprilTag predicted positions and yaw. AprilTag originally outputs the xyz and quaternion, but because we are dealing with F1/10 cars which lie in a 2D plane, we only need the yaw. Looking at the most recent output on the bottom of the terminal window, the apriltags2_ros library computes that the AprilTag is 0.165 meters to the left, 0.173 meters down, and 0.982 meters in front. It also predicts that the yaw of the tag relative to the camera is 0.466 degrees. I then measured the actual distance and yaw angle. Here is what I saw:
![measurement of actual AprilTag distance](https://github.com/mlab-upenn/future_pose_estimator/blob/master/photos/L1010427.jpg "measurement of actual AprilTag distance")

Measurement | Predicted Measurement | Actual Measurement | Error
----------- | --------------------- | ------------------ | -------
z-axis (front) | 0.982 meters | 0.940 meters | 4.28%
x-axis (left) | 0.173 meters | 0.206 meters | 16.0%
yaw (degrees) | 0.466 degrees | ~1 degree | 53.4%

- bag_future_pose_estimation.launch runs the near future pose estimation algorithm on a bag file. The bag file contains the laser scan, image raw stream, camera calibration, and vesc odometry which is used by the particle filter. Note that once the bag file starts playing, it takes around 5 seconds for apriltags2_ros to begin publishing transform messages, so you will want to press the space bar in the terminal to pause the bag file for a few seconds, so that the particle filter does not lose its localization. Here is a video of what you will see when you run this launch file: https://www.youtube.com/watch?v=HVJxHHeB9qo. 
- future_pose_estimation.launch is nearly identical to bag_future_pose_estimation.launch. The main difference is that instead of playing back from a bag file, the launch file activates racecar teleop.launch which allows the car to be driven live. When you are driving the car live, you may want to see what the car is seeing, with which you can set up VNC. Instructions are in this reference guide which I co-wrote: http://f1tenth.org/build.html#settingupvncserveronjetson. 

# Experimental Results
### Experiment 1. Accuracy of raw AprilTag measurements

### Experiment 2. Accuracy of near future pose predictions
(insert image of car going in the circle), need to explain what each color means. 

# Description of algorithm
Please reference the full paper I have written which is included in this repository. 
TODO: When I am done with the paper, include the pdf and .docx versions of it in the Github repository.

# Special Thanks
Thank you to Professor Mangharam and Professor Taylor for advising me on my senior thesis. I spent summer of 2018 learning about F1/10 cars for the first time, while developing a new ESE 680 course on autonomous racing which I assistant taught in fall of 2018. Then in spring of 2019 I worked on this project.






