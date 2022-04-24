#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallResponse

move = Twist()
value_front = 1
minimum_position = 0


def callback_scan(msg):

    global minimum_position
    minimum_position = 0
    minimum_value = msg.ranges[0]

    for i in range(720):
        if msg.ranges[i] < minimum_value:
            minimum_value = msg.ranges[i]
            minimum_position = i

    global value_front
    value_front = msg.ranges[360]
    # print("[scan] Value in front: ", value_front)
    print("[scan] Minimal position: ", minimum_position)


def callback_srv(request):

    print("[srv] Call Service Server")

    while abs(minimum_position - 360) > 15:
        print("[srv] Rotate")
        move.linear.x = 0
        move.angular.z = 0.25
        pub.publish(move)
        rospy.sleep(0.5)

    print("[srv] Wall is in front of the robot")

    while value_front > 0.3:
        print("[srv] Move forward")
        move.linear.x = 0.05
        move.angular.z = 0
        pub.publish(move)
        rospy.sleep(0.5)

    print("[srv] Wall is closer than 30cm")

    while abs(minimum_position - 180) > 15:
        move.linear.x = 0
        move.angular.z = 0.1
        pub.publish(move)
        rospy.sleep(0.5)

    print("[srv] Wall is on the right side")

    move.linear.x = 0
    move.angular.z = 0
    pub.publish(move)

    result = FindWallResponse()
    result.wallfound = True
    print("[srv] Service Server Finished")
    return result


rospy.init_node('find_wall_node')
rate = rospy.Rate(10)

sub = rospy.Subscriber('/scan', LaserScan, callback_scan)
srv = rospy.Service('/find_wall', FindWall, callback_srv)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

rospy.spin()
