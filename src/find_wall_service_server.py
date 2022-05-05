#! /usr/bin/env python3

import time
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallwalking.srv import FindWall, FindWallResponse


class FindWallService():

    move = Twist()
    value_front = int()
    minimum_position = int()

    def __init__(self):
        self.subScan = rospy.Subscriber('/scan', LaserScan, self.callback_scan)
        while self.subScan.get_num_connections() < 1:
            rospy.loginfo("[odom_as] Waiting for subsccription to /scan")
            time.sleep(0.1)

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.srv = rospy.Service('/find_wall', FindWall, self.callback_srv)
        self.rate = rospy.Rate(10)

    def callback_scan(self, msg):

        self.minimum_position = 0
        minimum_value = msg.ranges[0]

        for i in range(720):
            if msg.ranges[i] < minimum_value:
                minimum_value = msg.ranges[i]
                self.minimum_position = i

        self.value_front = msg.ranges[360]

        rospy.loginfo("[srv_scan] Minimum position: " +
                      str(self.minimum_position))

    def callback_srv(self, request):

        rospy.loginfo("[srv] Call Service Server")

        while abs(self.minimum_position - 360) > 15:
            rospy.loginfo("[srv] Rotate")
            self.move.linear.x = 0
            self.move.angular.z = 0.25
            self.pub.publish(self.move)
            rospy.sleep(0.5)

        rospy.loginfo("[srv] Wall is in front of the robot")

        while self.value_front > 0.3:
            rospy.loginfo("[srv] Move forward")
            self.move.linear.x = 0.05
            self.move.angular.z = 0
            self.pub.publish(self.move)
            rospy.sleep(0.5)

        rospy.loginfo("[srv] Wall is closer than 30cm")

        while abs(self.minimum_position - 180) > 15:
            self.move.linear.x = 0
            self.move.angular.z = 0.25
            self.pub.publish(self.move)
            rospy.sleep(0.5)

        rospy.loginfo("[srv] Wall is on the right side")

        self.move.linear.x = 0
        self.move.angular.z = 0
        self.pub.publish(self.move)

        result = FindWallResponse()
        result.wallfound = True
        rospy.loginfo("[srv] Service Server Finished")
        return result


if __name__ == '__main__':
    rospy.init_node('find_wall_node')
    FindWallService()
    rospy.spin()
