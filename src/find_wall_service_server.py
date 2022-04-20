#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallResponse


def callback_scan(msg):
    minimum_position = 0
    minimum_value = msg.ranges[0]

    for i in range(720):
        if msg.ranges[i] < minimum_value:
            minimum_value = msg.ranges[i]
            minimum_position = i

    global wall_in_front

    if minimum_position > 357 and minimum_position < 363:
        wall_in_front = True
    else:
        wall_in_front = False

    print("Wall in front: ", wall_in_front)


def callback_srv(request):

    global wall_in_front
    while not wall_in_front:
        move.linear.x = 0
        move.angular.z = 0.1
        pub.publish(move)

    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)

    result = FindWallResponse()
    result.wallfound = True

    return result


rospy.init_node('find_wall_node')
rate = rospy.Rate(10)
move = Twist()
wall_in_front = False

sub = rospy.Subscriber('/scan', LaserScan, callback_scan)
srv = rospy.Service('/find_wall', FindWall, callback_srv)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rospy.spin()
