#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallRequest


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


rospy.init_node("wall_walking_node")
rospy.wait_for_service('/find_wall')

find_wall_service = rospy.ServiceProxy('/find_wall', FindWall)
find_wall_request = FindWallRequest()
find_wall_result = find_wall_service(find_wall_request)
print(find_wall_result)

sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rate = rospy.Rate(1)
move = Twist()
move.linear.x = 0
move.angular.z = 0
pub.publish(move)

rospy.spin()
