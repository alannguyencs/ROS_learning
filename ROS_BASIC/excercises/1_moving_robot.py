#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('topic_publisher')
pub = rospy.Publisher('/cmd_vel_x', Twist, queue_size=1)
rate = rospy.Rate(2)
var = Twist()

"""
rosmsg show geometry_msgs/Twist
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
"""
var.linear.x = -0.1
# var.angular.z = 0.1

while not rospy.is_shutdown():
    pub.publish(var)
    print('var', var)
    rate.sleep()
