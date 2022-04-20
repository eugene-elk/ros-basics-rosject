#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    right = msg.ranges[180]
    print(right)


rospy.init_node("wall_walking_nod")
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(1)
move = Twist()
move.linear.x = 0
move.angular.z = 0
pub.publish(move)

rospy.spin()
