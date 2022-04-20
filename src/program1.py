#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def callback(msg):
    right = msg.ranges[270]
    front = msg.ranges[360]
    print("Front: ", front, ", Right: ", right)

    move.linear.x = 0.11
    move.angular.z = 0

    if right > 0.4:
        move.angular.z = -0.13
    elif right > 0.3:
        move.angular.z = 0
    else:
        move.angular.z = 0.13

    if front < 0.5:
        move.angular.z = 0.4

    pub.publish(move)


rospy.init_node("wall_walking_nod")
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(1)
move = Twist()
move.linear.x = 0
move.angular.z = 0
pub.publish(move)

rospy.spin()
