#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist


rospy.init_node("wall_walking_nod")
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(1)
move = Twist()
move.linear.x = 0
move.angular.z = 0
print("---------------STOP ROBOT-----------------------")
pub.publish(move)

rospy.spin()
