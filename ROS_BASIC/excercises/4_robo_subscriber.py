#! /usr/bin/env python
"""
Step 1: check some info

rostopic info /odom --> Type: nav_msgs/Odometry
rosmsg show nav_msgs/Odometry --> following info
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance

Step 2: compile and run the file
chmod +x robo_subcriber.py
cd ~/catkin_ws
catkin_make
source devel/setup.bash
rosrun my_subscriber_example_pkg robo_subcriber.py
"""

import rospy
from nav_msgs.msg import Odometry


def callback(msg):
    print(msg.twist)


rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
